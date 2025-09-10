#pragma once

#include "esphome/components/output/float_output.h"
#include "../stm32_port_expander.h"

namespace esphome {
namespace stm32_port_expander {

class Stm32PortExpanderFloatOutput : public output::FloatOutput {
 public:
  void set_parent(Stm32PortExpanderComponent *parent) {
    this->parent_ = parent;
  }
  void set_channel(uint8_t pin) {
    this->pin_ = pin;
  };

 protected:
  uint8_t pin_;
  Stm32PortExpanderComponent *parent_;
  void write_state(float state) override;
};

}  // namespace stm32_port_expander
}  // namespace esphome