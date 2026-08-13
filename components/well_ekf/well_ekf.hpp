#pragma once
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "eigen/Eigen/Dense"
#include <string>
#include <cmath>

namespace well_ekf {

using namespace Eigen;

class WellEKF : public PollingComponent, public sensor::Sensor {
private:
    Vector3f x;
    Matrix3f P;
    Matrix3f Q;
    float R;
    float A_well;
    bool ekf_initialized = false;
    uint32_t boot_time = 0;

    sensor::Sensor *level_sensor_{nullptr};
    sensor::Sensor *pump_flow_sensor_{nullptr};
    sensor::Sensor *permeability_sensor_{nullptr};
    
    std::string ha_h2_entity_;
    std::string ha_k_entity_;
    float init_h2 = NAN;
    float init_k = NAN;

public:
    void set_area(float area) { A_well = area; }
    void set_level_sensor(sensor::Sensor *sensor) { level_sensor_ = sensor; }
    void set_pump_flow_sensor(sensor::Sensor *sensor) { pump_flow_sensor_ = sensor; }
    void set_permeability_sensor(sensor::Sensor *sensor) { permeability_sensor_ = sensor; }
    void set_ha_h2_entity(const std::string &entity) { ha_h2_entity_ = entity; }
    void set_ha_k_entity(const std::string &entity) { ha_k_entity_ = entity; }

    void setup() override;
    void update() override;
};

} // namespace well_ekf