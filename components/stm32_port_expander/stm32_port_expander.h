#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

#define MAX_NUMBER_OF_PINS 28
#define MAX_NUMBER_OF_ERRORS 3

namespace esphome {
namespace stm32_port_expander {

class Stm32PortExpanderComponent : public Component, public i2c::I2CDevice {
 public:
  Stm32PortExpanderComponent() = default;

  void setup() override;
  /// Poll i2c
  void loop() override;
  /// Helper function to read the value of a pin.
  uint8_t read_pin_value(uint8_t pin);
  /// Helper function to write the value of a pin.
  void digital_write(uint8_t pin, bool value);
  /// Helper function to read the voltage of a pin.
  void analog_write(uint8_t pin, uint8_t value);

  float get_setup_priority() const override;

  void dump_config() override;

  void add_input_pin(uint8_t pin);

 private:
  bool enabled_pins_[MAX_NUMBER_OF_PINS];
  uint8_t pin_values_[MAX_NUMBER_OF_PINS];
  uint8_t number_of_errors_;
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




/*



class ArduinoPortExpanderSensor : public PollingComponent, public sensor::Sensor {
 public:
  void set_parent(ArduinoPortExpanderComponent *parent) {
    this->parent_ = parent;
  }

  void set_pin(uint8_t pin) {
    this->pin_ = pin;
  };

 protected:
  uint8_t pin_;
  ArduinoPortExpanderComponent *parent_;
  void update() override;
  float sample() override;
};






class ArduinoPortExpanderFloatOutput : public output::FloatOutput {
 public:
  void set_parent(ArduinoPortExpanderComponent *parent) {
    this->parent_ = parent;
  }
  void set_pin(uint8_t pin) {
    this->pin_ = pin;
  };

 protected:
  uint8_t pin_;
  ArduinoPortExpanderComponent *parent_;
  void write_state(float state) override;
};
*/

}  // namespace stm32_port_expander
}  // namespace esphome