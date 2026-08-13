#include "well_ekf.hpp"
#include "esphome/components/api/api_server.h"

namespace well_ekf {

void WellEKF::setup() {
    boot_time = millis();
    
    // Nediagonální matice Q není potřeba pro studnu, ale pro kinematiku by byla
    Q << 1e-6f, 0.0f, 0.0f,
         0.0f, 1e-9f, 0.0f,
         0.0f, 0.0f, 1e-12f;
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
    if (!level_sensor_->has_state()) return;

    float measured_h1 = level_sensor_->state;
    
    if (!ekf_initialized) {
        if (std::isnan(init_h2) || std::isnan(init_k)) {
            if (millis() - boot_time < 15000) return;
            if (std::isnan(init_h2)) init_h2 = measured_h1;
            if (std::isnan(init_k)) init_k = 0.0001f;
        }
        x << measured_h1, init_h2, init_k;
        P << 0.01f, 0.0f, 0.0f,
             0.0f, 0.1f, 0.0f,
             0.0f, 0.0f, 0.001f;
        ekf_initialized = true;
        return;
    }

    float dt_sec = this->get_update_interval() / 1000.0f;
    
    // Zjednodušený průtok
    float current_pump_flow = 0.0f; 

    // Predikce EKF (pro studnu)
    Vector3f x_pred;
    x_pred(0) = x(0) + dt_sec * (x(2) * (x(1) - x(0)) - (current_pump_flow / A_well));
    x_pred(1) = x(1);
    x_pred(2) = x(2);

    if (x_pred(0) < 0.0f) x_pred(0) = 0.0f;

    // Jacobiho matice pro studnu
    Matrix3f F;
    F << 1.0f - (dt_sec * x(2)),  dt_sec * x(2),  dt_sec * (x(1) - x(0)),
         0.0f,                    1.0f,           0.0f,
         0.0f,                    0.0f,           1.0f;

    Matrix3f P_pred = F * P * F.transpose() + Q;

    Matrix<float, 1, 3> H;
    H << 1.0f, 0.0f, 0.0f;

    float y = measured_h1 - x_pred(0);
    float S = (H * P_pred * H.transpose())(0, 0) + R;
    Vector3f K = P_pred * H.transpose() / S;

    x = x_pred + K * y;
    if (x(2) < 1e-6f) x(2) = 1e-6f;

    P = (Matrix3f::Identity() - K * H) * P_pred;
    P = 0.5f * (P + P.transpose());

    this->publish_state(x(1));
    if (permeability_sensor_ != nullptr) permeability_sensor_->publish_state(x(2));
}

} // namespace well_ekf