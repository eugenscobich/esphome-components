import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
  CONF_ID,
  CONF_PIN,
  CONF_RAW,
  DEVICE_CLASS_VOLTAGE,
  STATE_CLASS_MEASUREMENT,
  UNIT_VOLT,
)

ANALOG_PIN = {
  "A0": 0,
  "A1": 1,
  "A2": 2,
  "A3": 3,
  "A6": 6,
  "A7": 7,
}

AUTO_LOAD = ["voltage_sampler"]


Stm32PortExpanderSensor = stm32_port_expander_ns.class_(
  "ArduinoPortExpanderSensor",
  sensor.Sensor,
  cg.PollingComponent
)

CONFIG_SCHEMA = cv.All(
  sensor.sensor_schema(
    Stm32PortExpanderSensor
  )
  .extend(
    {
      cv.GenerateID(): cv.declare_id(Stm32PortExpanderSensor),
      cv.GenerateID(CONF_STM32_PORT_EXPANDER_ID): cv.use_id(
        Stm32PortExpanderComponent
      ),
      cv.Required(CONF_PIN): cv.All(
        cv.int_range(min=0, max=24)
      )
    }
  )
  .extend(cv.polling_component_schema("1s")),
  )


async def to_code(config):
  hub = await cg.get_variable(config[CONF_ARDUINO_PORT_EXPANDER_ID])

  var = cg.new_Pvariable(config[CONF_ID])
  await cg.register_component(var, config)
  await sensor.register_sensor(var, config)

  cg.add(var.set_pin(config[CONF_PIN]))
  cg.add(var.set_raw(config[CONF_RAW]))
  cg.add(var.set_parent(hub))