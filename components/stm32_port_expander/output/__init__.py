import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import output
from .. import (Stm32PortExpanderComponent, CONF_STM32_PORT_EXPANDER_ID, stm32_port_expander_ns)
from esphome.const import (
    CONF_ID,
    CONF_CHANNEL
)

DEPENDENCIES = ["stm32_port_expander"]

Stm32PortExpanderFloatOutput = stm32_port_expander_ns.class_(
    "Stm32PortExpanderFloatOutput", output.FloatOutput
)

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
  {
    cv.Required(CONF_ID): cv.declare_id(Stm32PortExpanderFloatOutput),
    cv.GenerateID(CONF_STM32_PORT_EXPANDER_ID): cv.use_id(Stm32PortExpanderComponent),
    cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=15),
  }
)

async def to_code(config):
  parent = await cg.get_variable(config[CONF_STM32_PORT_EXPANDER_ID])
  var = cg.new_Pvariable(config[CONF_ID])
  cg.add(var.set_channel(config[CONF_CHANNEL]))
  cg.add(var.set_parent(parent))
  await output.register_output(var, config)