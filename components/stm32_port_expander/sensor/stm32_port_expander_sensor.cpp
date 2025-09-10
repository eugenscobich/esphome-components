#include "stm32_port_expander_sensor.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

void Stm32PortExpanderSensor::set_pin(uint8_t pin) {
  this->pin_ = pin;
  this->parent_->add_input_pin(this->pin_);
};

void Stm32PortExpanderSensor::update() {
  uint8_t value = this->parent_->read_pin_value(this->pin_);
  this->publish_state(value / 100);
}

}  // namespace stm32_port_expander
}  // namespace esphome