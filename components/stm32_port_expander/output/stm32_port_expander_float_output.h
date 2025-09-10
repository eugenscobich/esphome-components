#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace stm32_port_expander {

class ArduinoPortExpanderFloatOutput : public output::FloatOutput {
 public:
  void set_parent(ArduinoPortExpanderComponent *parent) {
    this->parent_ = parent;
  }
  void set_channel(uint8_t pin) {
    this->pin_ = pin;
  };

 protected:
  uint8_t pin_;
  ArduinoPortExpanderComponent *parent_;
  void write_state(float state) override;
};

}  // namespace stm32_port_expander
}  // namespace esphome