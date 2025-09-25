#include "stm32_port_expander_float_output.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

void Stm32PortExpanderFloatOutput::write_state(float state) {
  this->parent_->analog_write(this->pin_, uint8_t(state * 255));
}


}  // namespace stm32_port_expander
}  // namespace esphome