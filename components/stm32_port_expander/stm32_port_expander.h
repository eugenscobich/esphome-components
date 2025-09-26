#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/gpio_expander/cached_gpio.h"

#define READ_ANALOG_INPUT_VALUES_CMD 0 // this cmd will be combined with pin number. Pins 0-15 will be analog in
#define WRITE_ANALOG_OUTPUT_VALUES_CMD 16
#define READ_DIGITAL_INPUT_VALUE_1_CMD 32
#define READ_DIGITAL_INPUT_VALUE_2_CMD 33
#define WRITE_DIGITAL_OUTPUT_VALUE_1_CMD 34
#define WRITE_DIGITAL_OUTPUT_VALUE_2_CMD 35
#define ACK_VALUE 130 // Like a password


namespace esphome {
namespace stm32_port_expander {

class Stm32PortExpanderComponent : public Component,
                                   public i2c::I2CDevice {
 public:
  Stm32PortExpanderComponent() = default;

  void setup() override;
  /// Poll i2c
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

  void pin_mode(uint8_t pin, gpio::Flags flags);

  void write_analog_output_value(uint8_t pin, uint8_t value);
  uint8_t read_analog_input_value(uint8_t pin);

  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);

 protected:

  bool read_gpio_1_();
  bool read_gpio_2_();
  bool write_gpio_1();
  bool write_gpio_2_();

  uint8_t digital_input_values[2] = {0x00};
  uint8_t digital_output_values[2] = {0x00};
 
  uint8_t analog_output_values_[16] = {0x00};
  uint8_t analog_input_values_[16] = {0x00};

  uint8_t write_data[2] = {0x00};
  uint8_t read_data[1] = {0x00};

};






/// Helper class to expose a Stm32PortExpander pin as an internal input GPIO pin.
class Stm32PortExpanderGPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;
  gpio::Flags get_flags() const override { return this->flags_; }

  void set_parent(Stm32PortExpanderComponent *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { this->flags_ = flags; }

 protected:
  Stm32PortExpanderComponent *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};



}  // namespace stm32_port_expander
}  // namespace esphome
