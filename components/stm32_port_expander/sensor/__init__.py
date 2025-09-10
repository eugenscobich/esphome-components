import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor
from .. import (Stm32PortExpanderComponent, CONF_STM32_PORT_EXPANDER_ID, stm32_port_expander_ns)
from esphome.const import (
    CONF_ID,
    CONF_PIN
)

DEPENDENCIES = ["stm32_port_expander"]

Stm32PortExpanderSensor = stm32_port_expander_ns.class_(
    "Stm32PortExpanderSensor", sensor.Sensor
)

CONFIG_SCHEMA = sensor.SENSOR_SCHEMA.extend(
  {
    cv.Required(CONF_ID): cv.declare_id(Stm32PortExpanderSensor),
    cv.GenerateID(CONF_STM32_PORT_EXPANDER_ID): cv.use_id(Stm32PortExpanderComponent),
    cv.Required(CONF_PIN): cv.int_range(min=0, max=29),
  }
)

async def to_code(config):
  parent = await cg.get_variable(config[CONF_STM32_PORT_EXPANDER_ID])
  var = cg.new_Pvariable(config[CONF_ID])
  cg.add(var.set_pin(config[CONF_PIN]))
  cg.add(var.set_parent(parent))
  await sensor.register_sensor(var, config)