#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace amis {


class AMISComponent : public Component, public uart::UARTDevice {
 public:
  void hex2bin(const std::string s, uint8_t *buf);
  uint8_t dif2len(uint8_t dif);
  void setup() override;
  void dump_config() override;
  void loop() override;
  void amis_decode();
  void set_power_grid_key(const std::string &power_grid_key);
  void set_energy_a_positive_sensor(sensor::Sensor *sensor) {
    this->energy_a_positive_sensor = sensor;
  }
  void set_energy_a_negative_sensor(sensor::Sensor *sensor) {
    this->energy_a_negative_sensor = sensor;
  }
  void set_reactive_energy_a_positive_sensor(sensor::Sensor *sensor) {
    this->reactive_energy_a_positive_sensor = sensor;
  }
  void set_reactive_energy_a_negative_sensor(sensor::Sensor *sensor) {
    this->reactive_energy_a_negative_sensor = sensor;
  }
  void set_instantaneous_power_a_positive_sensor(sensor::Sensor *sensor) {
    this->instantaneous_power_a_positive_sensor = sensor;
  }
  void set_instantaneous_power_a_negative_sensor(sensor::Sensor *sensor) {
    this->instantaneous_power_a_negative_sensor = sensor;
  }
  void set_reactive_instantaneous_power_a_positive_sensor(sensor::Sensor *sensor) {
    this->reactive_instantaneous_power_a_positive_sensor = sensor;
  }
  void set_reactive_instantaneous_power_a_negative_sensor(sensor::Sensor *sensor) {
    this->reactive_instantaneous_power_a_negative_sensor = sensor;
  }
  void set_timestamp_sensor(sensor::Sensor *sensor) {
    this->timestamp_sensor = sensor;
  }
  void set_voltage_l1_sensor(sensor::Sensor *sensor) {
    this->voltage_l1_sensor = sensor;
  }
  void set_voltage_l2_sensor(sensor::Sensor *sensor) {
    this->voltage_l2_sensor = sensor;
  }
  void set_voltage_l3_sensor(sensor::Sensor *sensor) {
    this->voltage_l3_sensor = sensor;
  }
  void set_current_l1_sensor(sensor::Sensor *sensor) {
    this->current_l1_sensor = sensor;
  }
  void set_current_l2_sensor(sensor::Sensor *sensor) {
    this->current_l2_sensor = sensor;
  }
  void set_current_l3_sensor(sensor::Sensor *sensor) {
    this->current_l3_sensor = sensor;
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  int bytes;
  uint8_t buffer[256];
  uint8_t decode_buffer[128];
  int expect;
  uint8_t iv[16];
  uint8_t key[16];
  sensor::Sensor *energy_a_positive_sensor{nullptr};
  sensor::Sensor *energy_a_negative_sensor{nullptr};
  sensor::Sensor *reactive_energy_a_positive_sensor{nullptr};
  sensor::Sensor *reactive_energy_a_negative_sensor{nullptr};
  sensor::Sensor *instantaneous_power_a_positive_sensor{nullptr};
  sensor::Sensor *instantaneous_power_a_negative_sensor{nullptr};
  sensor::Sensor *reactive_instantaneous_power_a_positive_sensor{nullptr};
  sensor::Sensor *reactive_instantaneous_power_a_negative_sensor{nullptr};
  sensor::Sensor *timestamp_sensor{nullptr};
  sensor::Sensor *voltage_l1_sensor{nullptr};
  sensor::Sensor *voltage_l2_sensor{nullptr};
  sensor::Sensor *voltage_l3_sensor{nullptr};
  sensor::Sensor *current_l1_sensor{nullptr};
  sensor::Sensor *current_l2_sensor{nullptr};
  sensor::Sensor *current_l3_sensor{nullptr};

};

}  // namespace rdm6300
}  // namespace esphome
