#include "stm32_port_expander.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace stm32_port_expander {

static const char *const TAG = "stm32_port_expander";

void Stm32PortExpanderComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Stm32PortExpander at 0x%02X", this->address_);

  if (!this->read_gpio_1_()) {
    ESP_LOGE(TAG, "Stm32PortExpander not available under 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  //this->write_gpio_();
  //this->read_gpio_();

}

void Stm32PortExpanderComponent::loop() {
	this->reset_pin_cache_();
}

void Stm32PortExpanderComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stm32PortExpander:");
    LOG_I2C_DEVICE(this);
    if (this->is_failed()) {
      ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    }
}

bool Stm32PortExpanderComponent::digital_read_hw(uint8_t pin) {
  if (pin >=0 && pin <=7) {
	this->read_gpio_1_();
	return this->digital_input_values_[0] & (1 << pin);
  } else {
	this->read_gpio_2_();
	return this->digital_input_values_[1] & (1 << pin - 8);
  }
  return false;
}

void Stm32PortExpanderComponent::digital_write_hw(uint8_t pin, bool value) {
	if (pin >=16 && pin <=23) {
	  if (value) {
		this->digital_output_values_[0]|= (1 << pin - 16);
	  } else {
		this->digital_output_values_[0] &= ~(1 << pin - 16);
	  }
	  this->write_gpio_1_();
	} else if (pin >=24 && pin <=31) {
	  if (value) {
		this->digital_output_values_[1]|= (1 << pin - 16);
	  } else {
		this->digital_output_values_[1] &= ~(1 << pin - 16);
	  }
	  this->write_gpio_2_();
	}
}

bool Stm32PortExpanderComponent::digital_read_cache(uint8_t pin) {
  if (pin >=0 && pin <=7) {
	this->read_gpio_1_();
	return this->digital_input_values_[0] & (1 << pin);
  } else {
	this->read_gpio_2_();
	return this->digital_input_values_[1] & (1 << pin - 8);
  }
}

void Stm32PortExpanderComponent::pin_mode(uint8_t pin, gpio::Flags flags) {

}

bool Stm32PortExpanderComponent::read_gpio_1_() {
  if (this->is_failed()) {
    return false;
  }
  //ESP_LOGI(TAG, "Stm32PortExpander read gpio 1");
  write_data[0] = READ_DIGITAL_INPUT_VALUE_1_CMD;
  write_data[1] = ACK_VALUE;
  if(this->write_read(write_data, 2, read_data, 1) == esphome::i2c::ERROR_OK) {
	  //ESP_LOGI(TAG, "Stm32PortExpander read value %d", read_data[0]);
	  digital_input_values_[0] = read_data[0];
	  this->status_clear_warning();
	  return true;
  }

  this->status_set_warning("Could not read digital input part one");
  return false;

}

bool Stm32PortExpanderComponent::read_gpio_2_() {
  if (this->is_failed()) {
    return false;
  }

  write_data[0] = READ_DIGITAL_INPUT_VALUE_2_CMD;
  write_data[1] = ACK_VALUE;
  if(this->write_read(write_data, 2, read_data, 1) == esphome::i2c::ERROR_OK) {
	  digital_input_values_[1] = read_data[0];
	  this->status_clear_warning();
	  return true;
  }

  this->status_set_warning("Could not read digital input part two");
  return false;

}

bool Stm32PortExpanderComponent::write_gpio_1_() {
  if (this->is_failed()) {
    return false;
  }

  write_data[0] = WRITE_DIGITAL_OUTPUT_VALUE_1_CMD;
  write_data[1] = digital_output_values_[0];
  if(this->write_read(write_data, 2, read_data, 1) == esphome::i2c::ERROR_OK) {
    this->status_clear_warning();
    return true;
  }

  this->status_set_warning("Could not write digital input part one");
  return false;
}

bool Stm32PortExpanderComponent::write_gpio_2_() {
  if (this->is_failed()) {
    return false;
  }

  write_data[0] = WRITE_DIGITAL_OUTPUT_VALUE_1_CMD;
  write_data[1] = digital_output_values_[1];
  if(this->write_read(write_data, 2, read_data, 1) == esphome::i2c::ERROR_OK) {
    this->status_clear_warning();
    return true;
  }

  this->status_set_warning("Could not write digital input part one");
  return false;
}

float Stm32PortExpanderComponent::get_setup_priority() const {
  return setup_priority::IO;
}

void Stm32PortExpanderComponent::write_analog_output_value(uint8_t pin, uint8_t value) {
    if (this->analog_output_values_[pin] != value) {
      if (this->is_failed()) {
		return;
	  }

	  write_data[0] = WRITE_ANALOG_OUTPUT_VALUES_CMD;
	  write_data[1] = value;
	  if(this->write_read(write_data, 2, read_data, 1) == esphome::i2c::ERROR_OK) {
		this->status_clear_warning();
	  }

	  this->status_set_warning("Could not write analog output value");
    }
    this->analog_output_values_[pin] = value;
}


uint8_t Stm32PortExpanderComponent::read_analog_input_value(uint8_t pin) {
  if (this->is_failed()) {
  	return 0;
  }

  write_data[0] = READ_ANALOG_INPUT_VALUES_CMD;
  write_data[1] = ACK_VALUE;
  if(this->write_read(write_data, 2, read_data, 1) == esphome::i2c::ERROR_OK) {
    this->status_clear_warning();
    this->analog_input_values_[pin] = read_data[0];
    return read_data[0];
  }

  this->status_set_warning("Could not read analog input value");
  return 0;
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
