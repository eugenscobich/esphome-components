#include "stm32_port_expander.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

static const char *const TAG = "stm32_port_expander";

static const uint32_t CONFIGURE_TIMEOUT_MS = 3000;

static const uint8_t CMD_ACK = 0; // Read value from first pin

float Stm32PortExpanderComponent::get_setup_priority() const {
  return setup_priority::IO;
}

void Stm32PortExpanderComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Stm32PortExpander at %#02x", this->address_);
}

void Stm32PortExpanderComponent::loop() {
  for (uint8_t pin = 0; pin < MAX_NUMBER_OF_PINS; pin++) {
    if (enabled_pins_[pin]) {
      if (i2c::ERROR_OK != this->read_register(pin, this->pin_values_ + pin, 1)) {
        ESP_LOGE(TAG, "Error reading digital input at pin[%d].", pin);
        return;
      }
      ESP_LOGD(TAG, "Successful received binary sensor value[%d] for pin[%d].", this->pin_values_[pin], pin);
    }
  }
}

void Stm32PortExpanderComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Stm32PortExpander:");
  LOG_I2C_DEVICE(this)
}

uint8_t Stm32PortExpanderComponent::read_pin_value(uint8_t pin) {
  return this->pin_values_[pin];
}

void Stm32PortExpanderComponent::digital_write(uint8_t pin, bool value) {
  uint8_t valueToSend = uint8_t(value);
  bool success = (this->write_register(pin, &valueToSend, 1) == i2c::ERROR_OK);
  if (!success) {
    ESP_LOGW(TAG, "Could not write digital value %d to pin %d", valueToSend, pin);
    this->status_set_warning();
    return;
  }
}

uint8_t Stm32PortExpanderComponent::analog_read(uint8_t pin) {
  uint8_t value;
  bool success = this->read_register(pin, &value, 1);
  if (!success) {
    ESP_LOGW(TAG, "Could not read analog value for pin %d", pin);
    this->status_set_warning();
    return 0;
  }
  ESP_LOGV(TAG, "Analog read pin: %d, success: %d, value %d", pin, success, value);
  return value;
}

void Stm32PortExpanderComponent::analog_write(uint8_t pin, uint8_t value) {
  bool success = this->write_register(pin, &value, 1);
  if (!success) {
    ESP_LOGW(TAG, "Could not write analog value %d to pin %d", value, pin);
    this->status_set_warning();
    return;
  }
}


void Stm32PortExpanderComponent::add_input_pin(uint8_t pin) {
  ESP_LOGCONFIG(TAG, "Adding pin %d as input", pin);
  this->pin_values_[pin] = true;
}







void Stm32PortExpanderGPIOPin::setup() {
   ESP_LOGCONFIG(TAG, "Setting up Stm32PortExpanderGPIOPin");
   if (this->flags_ == FLAG_INPUT) {
     this->parent_->add_input_pin(this->pin_);
   }
}

void Stm32PortExpanderGPIOPin::pin_mode(gpio::Flags flags) {
   ESP_LOGCONFIG(TAG, "Setting up pin mode for Stm32PortExpanderGPIOPin");
   this->flags_ = flags
}

bool Stm32PortExpanderGPIOPin::digital_read() {
  return this->parent_->digital_read(this->pin_) != this->inverted_;
}

void Stm32PortExpanderGPIOPin::digital_write(bool value) {
    this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string Stm32PortExpanderGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via Stm32PortExpander", pin_);
  return buffer;
}

/*


float ArduinoPortExpanderSensor::sample() {
  return this->parent_->analog_read(this->pin_);
}

void ArduinoPortExpanderSensor::update() {
  this->publish_state(this->sample());
}

void ArduinoPortExpanderFloatOutput::write_state(float state) {
  this->parent_->analog_write(this->pin_, state);
}
*/

}  // namespace stm32_port_expander
}  // namespace esphome