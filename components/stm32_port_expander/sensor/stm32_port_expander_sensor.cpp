#include "stm32_port_expander_sensor.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

void Stm32PortExpanderSensor::setup() {
  //this->parent_->add_input_pin(this->pin_);
};


void Stm32PortExpanderSensor::set_pin(uint8_t pin) {
  this->pin_ = pin;
};

void Stm32PortExpanderSensor::update() {
  uint8_t value = this->parent_->read_analog_input_value(this->pin_);
  this->publish_state(value);
}

}  // namespace stm32_port_expander
}  // namespace esphome