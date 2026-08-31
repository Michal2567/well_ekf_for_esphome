#include "well_ekf.hpp"
#include <cstring>
#include <cmath>

namespace well_ekf {

void WellEKF::setup() {
    boot_time = esphome::millis();
    
    memset(Q, 0, sizeof(Q));
    Q[0][0] = 1e-8f;
    Q[1][1] = 1e-14f;
    Q[2][2] = 1e-14f;
    
    R = 0.0001f;
}

void WellEKF::update() {
    if (!level_sensor_->has_state() || !pump_flow_sensor_->has_state()) return;

    float measured_h1 = level_sensor_->state;
    float current_pump_flow = pump_flow_sensor_->state; 
    
    if (!ekf_initialized) {
        if (std::isnan(init_h2) && init_h2_sensor_->has_state()) {
            init_h2 = init_h2_sensor_->state;
        }
        if (std::isnan(init_k) && init_k_sensor_->has_state()) {
            init_k = init_k_sensor_->state;
        }

        if (std::isnan(init_h2) || std::isnan(init_k)) {
            if (esphome::millis() - boot_time < 15000) return;
            if (std::isnan(init_h2)) init_h2 = measured_h1;
            if (std::isnan(init_k)) init_k = 0.0001f;
        }
        
        x[0] = measured_h1;
        x[1] = init_h2;
        x[2] = init_k;
        
        memset(P, 0, sizeof(P));
        P[0][0] = 1e-10f;
        P[1][1] = 1e-5f;
        P[2][2] = 1e-5f;
        
        ekf_initialized = true;
        return;
    }

    float dt_sec = this->get_update_interval() / 1000.0f;
    
    float x_pred[3];
    x_pred[0] = x[0] + dt_sec * (x[2] * (x[1] - x[0]) - (current_pump_flow / A_well));
    x_pred[1] = x[1];
    x_pred[2] = x[2];
    if (x_pred[0] < 0.0f) x_pred[0] = 0.0f;

    float F[3][3];
    memset(F, 0, sizeof(F));
    F[0][0] = 1.0f - (dt_sec * x[2]);
    F[0][1] = dt_sec * x[2];
    F[0][2] = dt_sec * (x[1] - x[0]);
    F[1][1] = 1.0f;
    F[2][2] = 1.0f;

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
            for (int k = 0; k < 3; k++) P_pred[i][j] += FP[i][k] * F[j][k];
        }
    }

    float y = measured_h1 - x_pred[0];
    float S = P_pred[0][0] + R;
    
    float K[3];
    K[0] = P_pred[0][0] / S;
    K[1] = P_pred[1][0] / S;
    K[2] = P_pred[2][0] / S;

    for (int i = 0; i < 3; i++) {
        x[i] = x_pred[i] + K[i] * y;
    }
    if (x[2] < 1e-6f) x[2] = 1e-6f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = P_pred[i][j] - K[i] * P_pred[0][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float avg = (P[i][j] + P[j][i]) * 0.5f;
            P[i][j] = P[j][i] = avg;
        }
    }

    this->publish_state(x[1]);
    if (permeability_sensor_ != nullptr) permeability_sensor_->publish_state(x[2]);
    if (filtered_h1_sensor_ != nullptr) filtered_h1_sensor_->publish_state(x[0]);
    if (innovation_sensor_ != nullptr) innovation_sensor_->publish_state(y);
    if (p_h1_sensor_ != nullptr) p_h1_sensor_->publish_state(P[0][0]);
    if (p_h2_sensor_ != nullptr) p_h2_sensor_->publish_state(P[1][1]);
    if (p_k_sensor_ != nullptr) p_k_sensor_->publish_state(P[2][2]);
}

} // namespace well_ekf