#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/preferences.h"

namespace well_ekf {

struct EKFSaveState {
  float x[3];
  float P[3][3];
} __attribute__((packed));

class WellEKF : public esphome::PollingComponent, public esphome::sensor::Sensor {
 public:
  void setup() override;
  void update() override;
  void on_shutdown() override;

  void set_level_sensor(esphome::sensor::Sensor *sensor) { level_sensor_ = sensor; }
  void set_pump_flow_sensor(esphome::sensor::Sensor *sensor) { pump_flow_sensor_ = sensor; }
  
  void set_permeability_sensor(esphome::sensor::Sensor *sensor) { permeability_sensor_ = sensor; }
  void set_filtered_h1_sensor(esphome::sensor::Sensor *sensor) { filtered_h1_sensor_ = sensor; }
  void set_innovation_sensor(esphome::sensor::Sensor *sensor) { innovation_sensor_ = sensor; }
  void set_p_h1_sensor(esphome::sensor::Sensor *sensor) { p_h1_sensor_ = sensor; }
  void set_p_h2_sensor(esphome::sensor::Sensor *sensor) { p_h2_sensor_ = sensor; }
  void set_p_k_sensor(esphome::sensor::Sensor *sensor) { p_k_sensor_ = sensor; }

  void set_area(float area) { A_well = area; }

  float A_well{1.13f};

 protected:
  esphome::sensor::Sensor *level_sensor_{nullptr};
  esphome::sensor::Sensor *pump_flow_sensor_{nullptr};
  
  esphome::sensor::Sensor *permeability_sensor_{nullptr};
  esphome::sensor::Sensor *filtered_h1_sensor_{nullptr};
  esphome::sensor::Sensor *innovation_sensor_{nullptr};
  esphome::sensor::Sensor *p_h1_sensor_{nullptr};
  esphome::sensor::Sensor *p_h2_sensor_{nullptr};
  esphome::sensor::Sensor *p_k_sensor_{nullptr};

  esphome::ESPPreferenceObject pref_;
  uint32_t last_save_time_{0};

  bool ekf_initialized{false};
  uint32_t boot_time{0};
  
  float x[3];
  float P[3][3];
  float Q[3][3];
  float R;
};

} // namespace well_ekf