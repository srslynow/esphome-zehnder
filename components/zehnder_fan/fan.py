import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import fan, spi

DEPENDENCIES = ["spi"]

CONF_RADIO_ID = "radio_id"
CONF_PWR_PIN = "pwr_pin"
CONF_CE_PIN = "ce_pin"
CONF_TXEN_PIN = "txen_pin"
CONF_DR_PIN = "dr_pin"

zehnder_fan_ns = cg.esphome_ns.namespace("zehnder_fan")
NRF905Controller = zehnder_fan_ns.class_("NRF905Controller", spi.SPIDevice)
ZehnderFanComponent = zehnder_fan_ns.class_(
    "ZehnderFanComponent", fan.Fan, cg.Component
)

CONFIG_SCHEMA = (
    # The ventilation unit keeps its speed while the controller restarts, so by default
    # assume it is still doing what we last told it to do.
    fan.fan_schema(ZehnderFanComponent, default_restore_mode="RESTORE_DEFAULT_OFF")
    .extend(
        {
            cv.GenerateID(CONF_RADIO_ID): cv.declare_id(NRF905Controller),
            cv.Required(CONF_PWR_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_TXEN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DR_PIN): pins.gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    radio = cg.new_Pvariable(config[CONF_RADIO_ID])
    await spi.register_spi_device(radio, config)
    for conf_pin, setter in (
        (CONF_PWR_PIN, radio.set_pwr_pin),
        (CONF_CE_PIN, radio.set_ce_pin),
        (CONF_TXEN_PIN, radio.set_txen_pin),
        (CONF_DR_PIN, radio.set_dr_pin),
    ):
        pin = await cg.gpio_pin_expression(config[conf_pin])
        cg.add(setter(pin))

    var = await fan.new_fan(config, radio)
    await cg.register_component(var, config)
