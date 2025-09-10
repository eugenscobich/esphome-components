#pragma once

#include "esphome/components/sensor/sensor.h"
#include "../stm32_port_expander.h"

namespace esphome {
namespace stm32_port_expander {

class Stm32PortExpanderSensor : public sensor::Sensor {
 public:
  void set_parent(Stm32PortExpanderComponent *parent) {
    this->parent_ = parent;
  }
  void set_pin(uint8_t pin);

  void update() override;
  float sample() override;

 protected:
  uint8_t pin_;
  Stm32PortExpanderComponent *parent_;
};

}  // namespace stm32_port_expander
}  // namespace esphome