#include "well_ekf.hpp"
#include "esphome/components/api/api_server.h"
#include <cstring>

namespace well_ekf {

void WellEKF::setup() {
    boot_time = millis();
    
    // Inicializace Q matice nulami a nastavení diagonály
    memset(Q, 0, sizeof(Q));
    Q[0][0] = 1e-6f;
    Q[1][1] = 1e-9f;
    Q[2][2] = 1e-12f;
    
    R = 0.0001f;
    
    if (esphome::api::global_api_server != nullptr) {
        esphome::api::global_api_server->subscribe_home_assistant_state(ha_h2_entity_, [this](std::string state) {
            auto val = parse_number<float>(state);
            if (val.has_value()) this->init_h2 = val.value();
        });
        esphome::api::global_api_server->subscribe_home_assistant_state(ha_k_entity_, [this](std::string state) {
            auto val = parse_number<float>(state);
            if (val.has_value()) this->init_k = val.value();
        });
    }
}

void WellEKF::update() {
    if (!level_sensor_->has_state() || !pump_flow_sensor_->has_state()) return;

    float measured_h1 = level_sensor_->state;
    float current_pump_flow = pump_flow_sensor_->state; 
    
    if (!ekf_initialized) {
        if (std::isnan(init_h2) || std::isnan(init_k)) {
            // Cekani az 15 vterin na data z HA
            if (millis() - boot_time < 15000) return;
            if (std::isnan(init_h2)) init_h2 = measured_h1;
            if (std::isnan(init_k)) init_k = 0.0001f;
        }
        
        x[0] = measured_h1;
        x[1] = init_h2;
        x[2] = init_k;
        
        memset(P, 0, sizeof(P));
        P[0][0] = 0.01f;
        P[1][1] = 0.1f;
        P[2][2] = 0.001f;
        
        ekf_initialized = true;
        return;
    }

    float dt_sec = this->get_update_interval() / 1000.0f;
    
    // 1. Predikce stavu (x_pred)
    float x_pred[3];
    x_pred[0] = x[0] + dt_sec * (x[2] * (x[1] - x[0]) - (current_pump_flow / A_well));
    x_pred[1] = x[1];
    x_pred[2] = x[2];
    if (x_pred[0] < 0.0f) x_pred[0] = 0.0f;

    // 2. Vytvoření Jacobiho matice (F)
    float F[3][3];
    memset(F, 0, sizeof(F));
    F[0][0] = 1.0f - (dt_sec * x[2]);
    F[0][1] = dt_sec * x[2];
    F[0][2] = dt_sec * (x[1] - x[0]);
    F[1][1] = 1.0f;
    F[2][2] = 1.0f;

    // 3. Predikce kovariance: P_pred = F * P * F^T + Q
    float FP[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0;
            for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * P[k][j];
        }
    }
    
    float P_pred[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = Q[i][j];
            for (int k = 0; k < 3; k++) P_pred[i][j] += FP[i][k] * F[j][k]; // F[j][k] odpovida transpozici F^T
        }
    }

    // 4. Inovace a Kalmanův zisk
    // Matice H = [1, 0, 0], coz znamena ze merime pouze h1 (x_pred[0])
    float y = measured_h1 - x_pred[0];
    
    // Inovacni kovariance S = H * P_pred * H^T + R -> odpovida prvnimu prvku matice
    float S = P_pred[0][0] + R;
    
    // Kalmanuv zisk K = P_pred * H^T / S -> odpovida prvnimu sloupci matice P_pred vydelenemu S
    float K[3];
    K[0] = P_pred[0][0] / S;
    K[1] = P_pred[1][0] / S;
    K[2] = P_pred[2][0] / S;

    // 5. Aktualizace stavu: x = x_pred + K * y
    for (int i = 0; i < 3; i++) {
        x[i] = x_pred[i] + K[i] * y;
    }
    if (x[2] < 1e-6f) x[2] = 1e-6f; // Ochrana proti zaporne propustnosti

    // 6. Aktualizace kovariance: P = (I - K * H) * P_pred
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // K * H vytvori matici, kde nenulovy je pouze prvni sloupec, coz rovnici vyrazne zjednodusi
            P[i][j] = P_pred[i][j] - K[i] * P_pred[0][j];
        }
    }

    // Symetrizace matice P k zamezeni akumulace chyb v plovouci radove carce
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float avg = (P[i][j] + P[j][i]) * 0.5f;
            P[i][j] = P[j][i] = avg;
        }
    }

    // Odeslani vysledku
    this->publish_state(x[1]);
    if (permeability_sensor_ != nullptr) permeability_sensor_->publish_state(x[2]);
}

} // namespace well_ekf