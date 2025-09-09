import esphome.codegen as cg
import esphome.config_validation as cv

from esphome import pins
from esphome.components import i2c, sensor, output
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_NUMBER,
    CONF_MODE,
    CONF_INVERTED,
    CONF_OUTPUT,
    CONF_PULLUP,
)

CONF_STM32_PORT_EXPANDER = "stm32_port_expander"
CONF_STM32_PORT_EXPANDER_ID = "stm32_port_expander_id"

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

stm32_port_expander_ns = cg.esphome_ns.namespace("stm32_port_expander")

Stm32PortExpanderComponent = stm32_port_expander_ns.class_(
    "Stm32PortExpanderComponent", cg.Component, i2c.I2CDevice
)

Stm32PortExpanderGPIOPin = stm32_port_expander_ns.class_(
    "Stm32PortExpanderGPIOPin", cg.GPIOPin
)


CONF_Stm32PortExpander = "stm32_port_expander"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(Stm32PortExpanderComponent)
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x08))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


STM32_PORT_EXPANDER_PIN_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(Stm32PortExpanderGPIOPin),
        cv.Required(CONF_Stm32PortExpander): cv.use_id(Stm32PortExpanderComponent),
        cv.Required(CONF_NUMBER): cv.All(
            cv.int_range(min=0, max=24)
        ),
        cv.Optional(CONF_MODE, default={}): cv.All(
            {
                cv.Optional(CONF_INPUT, default=False): cv.boolean,
                cv.Optional(CONF_OUTPUT, default=False): cv.boolean,
                cv.Optional(CONF_PULLUP, default=False): cv.boolean,
            },
            validate_mode,
        ),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)


@pins.PIN_SCHEMA_REGISTRY.register(
    CONF_STM32_PORT_EXPANDER, STM32_PORT_EXPANDER_PIN_SCHEMA
)
async def Stm32PortExpander_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_STM32_PORT_EXPANDER])

    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var