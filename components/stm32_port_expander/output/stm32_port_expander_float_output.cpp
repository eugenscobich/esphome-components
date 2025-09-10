#include "stm32_port_expander.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

void ArduinoPortExpanderFloatOutput::write_state(float state) {
  this->parent_->analog_write(this->pin_, uint8_t(state * 100));
}


}  // namespace stm32_port_expander
}  // namespace esphome