#include "stm32_port_expander.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

static const char *const TAG = "stm32_port_expander";

static const uint32_t CONFIGURE_TIMEOUT_MS = 5000;

static const uint8_t CMD_ACK = 125; //01010101
static const uint8_t CMD_DIGITAL_PORT_A = 100;
static const uint8_t CMD_DIGITAL_PORT_B = 101;
static const uint8_t CMD_DIGITAL_PORT_C = 102;

float Stm32PortExpanderComponent::get_setup_priority() const {
  return setup_priority::IO;
}

void Stm32PortExpanderComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Stm32PortExpander at %#02x ...", this->address_);
  uint8_t data[1];
  uint32_t timeout = millis() + CONFIGURE_TIMEOUT_MS;

  while (this->read_register(CMD_ACK, data, 1) != i2c::ERROR_OK) {
    App.feed_wdt();
    if (millis() > timeout) {
      ESP_LOGE(TAG, "Stm32PortExpander not available at 0x%02X", this->address_);
      this->mark_failed();
      return;
    }
  };
  if (data[0] == 125) {
    ESP_LOGCONFIG(TAG, "Successfully configured.");
  } else {
    ESP_LOGE(TAG, "Stm32PortExpander acknowledgment message is wrong, it must be 01010101");
  }
}

void Stm32PortExpanderComponent::loop() {
    bool success = (this->read_register(CMD_DIGITAL_PORT_A, this->read_buffer_, 1) == i2c::ERROR_OK);
    if (!success) {
      ESP_LOGW(TAG, "Could not read digital values for port A");
      this->status_set_warning();
      return;
    }
    success = (this->read_register(CMD_DIGITAL_PORT_B, this->read_buffer_ + 8, 1) == i2c::ERROR_OK);
    if (!success) {
      ESP_LOGW(TAG, "Could not read digital values for port B");
      this->status_set_warning();
      return;
    }
    success = (this->read_register(CMD_DIGITAL_PORT_B, this->read_buffer_ + 16, 1) == i2c::ERROR_OK);
    if (!success) {
      ESP_LOGW(TAG, "Could not read digital values for port C");
      this->status_set_warning();
      return;
    }
    this->status_clear_warning();
}

void Stm32PortExpanderComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Stm32PortExpander:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with Stm32PortExpander failed!");
  }
}

bool Stm32PortExpanderComponent::digital_read(uint8_t pin) {
  uint8_t bit = pin % 8;
  uint8_t value = pin < 8 ? this->read_buffer_[0] : pin < 16 ? this->read_buffer_[1] : this->read_buffer_[2];
  return value & (1 << bit);
}

void Stm32PortExpanderComponent::digital_write(uint8_t pin, bool value) {
  if (this->is_failed()) {
    return;
  }
  uint8_t valueToSend = uint8_t(value);
  this->write_register(pin, &valueToSend, 1);
}

float Stm32PortExpanderComponent::analog_read(uint8_t pin) {
  uint8_t value;
  bool success = (this->read_register(pin, &value, 1));
  if (!success) {
    ESP_LOGW(TAG, "Could not read analog value for pin %d", pin);
    this->status_set_warning();
    return 0;
  }
  ESP_LOGV(TAG, "Analog read pin: %d, success: %d, value %d", pin, success, value);
  return value / 255;
}

void Stm32PortExpanderComponent::analog_write(uint8_t pin, float value) {
  if (this->is_failed()) {
    return;
  }
  uint8_t valueToSend = uint8_t(value * 255);
  this->write_register(pin, &valueToSend, 1);
}


/*
void Stm32PortExpanderGPIOPin::setup() {
  this->setup_ = true;
}
bool Stm32PortExpanderGPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void Stm32PortExpanderGPIOPin::digital_write(bool value) {
  if (this->setup_)
    this->parent_->digital_write(this->pin_, value != this->inverted_);
}
std::string Stm32PortExpanderGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via Stm32PortExpander", pin_);
  return buffer;
}




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