#pragma once
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <cmath>

namespace well_ekf {

class WellEKF : public esphome::PollingComponent, public esphome::sensor::Sensor {
private:
    float x[3];
    float P[3][3];
    float Q[3][3];
    float R;
    float A_well;
    bool ekf_initialized = false;
    uint32_t boot_time = 0;

    esphome::sensor::Sensor *level_sensor_{nullptr};
    esphome::sensor::Sensor *pump_flow_sensor_{nullptr};
    esphome::sensor::Sensor *init_h2_sensor_{nullptr};
    esphome::sensor::Sensor *init_k_sensor_{nullptr};
    esphome::sensor::Sensor *permeability_sensor_{nullptr};
    
    float init_h2 = NAN;
    float init_k = NAN;

public:
    void set_area(float area) { A_well = area; }
    void set_level_sensor(esphome::sensor::Sensor *sensor) { level_sensor_ = sensor; }
    void set_pump_flow_sensor(esphome::sensor::Sensor *sensor) { pump_flow_sensor_ = sensor; }
    void set_init_h2_sensor(esphome::sensor::Sensor *sensor) { init_h2_sensor_ = sensor; }
    void set_init_k_sensor(esphome::sensor::Sensor *sensor) { init_k_sensor_ = sensor; }
    void set_permeability_sensor(esphome::sensor::Sensor *sensor) { permeability_sensor_ = sensor; }

    void setup() override;
    void update() override;
};

} // namespace well_ekf#pragma once
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <cmath>

namespace well_ekf {

class WellEKF : public esphome::PollingComponent, public esphome::sensor::Sensor {
private:
    float x[3];
    float P[3][3];
    float Q[3][3];
    float R;
    float A_well;
    bool ekf_initialized = false;
    uint32_t boot_time = 0;

    esphome::sensor::Sensor *level_sensor_{nullptr};
    esphome::sensor::Sensor *pump_flow_sensor_{nullptr};
    esphome::sensor::Sensor *init_h2_sensor_{nullptr};
    esphome::sensor::Sensor *init_k_sensor_{nullptr};
    esphome::sensor::Sensor *permeability_sensor_{nullptr};
    
    float init_h2 = NAN;
    float init_k = NAN;

public:
    void set_area(float area) { A_well = area; }
    void set_level_sensor(esphome::sensor::Sensor *sensor) { level_sensor_ = sensor; }
    void set_pump_flow_sensor(esphome::sensor::Sensor *sensor) { pump_flow_sensor_ = sensor; }
    void set_init_h2_sensor(esphome::sensor::Sensor *sensor) { init_h2_sensor_ = sensor; }
    void set_init_k_sensor(esphome::sensor::Sensor *sensor) { init_k_sensor_ = sensor; }
    void set_permeability_sensor(esphome::sensor::Sensor *sensor) { permeability_sensor_ = sensor; }

    void setup() override;
    void update() override;
};

} // namespace well_ekf