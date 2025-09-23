#include "stm32_port_expander.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

static const char *const TAG = "stm32_port_expander";

void Stm32PortExpanderComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Stm32PortExpander at 0x%02X", this->address_);

  if (!this->read_gpio_()) {
    ESP_LOGE(TAG, "Stm32PortExpander not available under 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  this->write_gpio_();
  this->read_gpio_();

}

void Stm32PortExpanderComponent::loop() {

  for (uint8_t i = 0; i < 16; i++) {
    if (this->channels_needs_update_mask_ & (1 << i) > 0) {
      ESP_LOGV(TAG, "Pin[%d] requires update.", i);

      uint8_t data[2];
      data[0] = i;
      data[1] = this->analog_output_values_[i];

      bool success;
      success = this->write_bytes(WRITE_ANALOG_OUTPUT_VALUES_CMD, data, 2);
      if (!success) {
        this->status_set_warning();
        return;
      }
    }
  }

  this->reset_pin_cache_();
}

void Stm32PortExpanderComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stm32PortExpander:");
    LOG_I2C_DEVICE(this)
    if (this->is_failed()) {
      ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    }
}

bool Stm32PortExpanderComponent::digital_read(uint8_t pin) {
  return this->read_gpio_();  // Return true if I2C read succeeded, false on error
  return this->digital_input_values_ & (1 << pin);
}

void Stm32PortExpanderComponent::digital_write_hw(uint8_t pin, bool value) {
  if (value) {
    this->digital_output_values_ |= (1 << pin);
  } else {
    this->digital_output_values_ &= ~(1 << pin);
  }
  this->write_gpio_();
}


void Stm32PortExpanderComponent::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT) {
    // Clear mode mask bit
    this->digital_mode_mask_ &= ~(1 << pin);
    // Write GPIO to enable input mode
    this->write_gpio_();
  } else if (flags == gpio::FLAG_OUTPUT) {
    // Set mode mask bit
    this->digital_mode_mask_ |= 1 << pin;
  }
}

bool Stm32PortExpanderComponent::read_gpio_() {
  if (this->is_failed())
    return false;
  bool success;
  uint8_t data[2];
  success = this->read_bytes(READ_DIGITAL_INPUT_VALUES_CMD, data, 2);
  this->digital_input_values_ = (uint16_t(data[1]) << 8) | (uint16_t(data[0]) << 0);
  if (!success) {
    this->status_set_warning();
    return false;
  }
  this->status_clear_warning();
  return true;
}

bool Stm32PortExpanderComponent::write_gpio_() {
  if (this->is_failed())
    return false;

  uint16_t value = 0;
  // Pins in OUTPUT mode and where pin is HIGH.
  value |= this->digital_mode_mask_ & this->digital_output_values_;
  // Pins in INPUT mode must also be set here
  value |= ~this->digital_mode_mask_;

  uint8_t data[2];
  data[0] = value;
  data[1] = value >> 8;
  bool success;
  success = this->write_bytes(WRITE_DIGITAL_OUTPUT_VALUES_CMD, data, 2);
  if (!success) {
    this->status_set_warning();
    return false;
  }

  this->status_clear_warning();
  return true;
}

float Stm32PortExpanderComponent::get_setup_priority() const {
  return setup_priority::IO;
}

// Run our loop() method early to invalidate cache before any other components access the pins
float Stm32PortExpanderComponent::get_loop_priority() const {
  return 9.0f;
}  // Just after WIFI


void Stm32PortExpanderComponent::analog_write(uint8_t channel, uint8_t value) {
    if (this->analog_output_values_[channel] != value) {
      this->channels_needs_update_mask_ |= 1 << channel;
    }
    this->analog_output_values_[channel] = value;
}


uint8_t Stm32PortExpanderComponent::read_analog_input_value(uint8_t channel) {
  if (this->is_failed()) {
    return false;
  }
  bool success;
  uint8_t data[1];
  success = this->read_bytes(READ_ANALOG_INPUT_VALUES_CMD + channel, data, 1);
  if (!success) {
    this->status_set_warning();
    return false;
  }
  this->analog_input_values_[channel] = data[0];
  this->status_clear_warning();
  return data[0];
}


void Stm32PortExpanderGPIOPin::setup() {
   pin_mode(flags_);
}

void Stm32PortExpanderGPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
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

}  // namespace stm32_port_expander
}  // namespace esphome