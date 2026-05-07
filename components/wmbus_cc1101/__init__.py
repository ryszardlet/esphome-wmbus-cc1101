"""ESPHome external component: wmbus_cc1101.

Hub-style component that drives a TI CC1101 radio over SPI for wM-Bus T1
reception at 868.95 MHz. Sensor entities are registered separately under
``sensor:`` (see sensor.py).
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_FREQUENCY, CONF_ID

CODEOWNERS = ["@ryszardlet"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = False

CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_CLK_PIN = "clk_pin"
CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_GDO2_PIN = "gdo2_pin"
CONF_LOG_UNKNOWN = "log_unknown"

wmbus_cc1101_ns = cg.esphome_ns.namespace("wmbus_cc1101")
WMBusComponent = wmbus_cc1101_ns.class_("WMBusComponent", cg.Component)


def _frequency_mhz(value):
    # Accept "868.950", "868.950 MHz", or a number in MHz; return Hz as int.
    if isinstance(value, str):
        s = value.strip().lower().replace("mhz", "").strip()
        try:
            mhz = float(s)
        except ValueError as err:
            raise cv.Invalid(f"Invalid frequency: {value!r}") from err
    else:
        mhz = float(value)
    if not 300.0 <= mhz <= 928.0:
        raise cv.Invalid("frequency must be a CC1101 RF band value in MHz")
    return int(round(mhz * 1_000_000))


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WMBusComponent),
        cv.Required(CONF_MOSI_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_MISO_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_CLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_CS_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_GDO0_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_GDO2_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_FREQUENCY, default="868.950"): _frequency_mhz,
        cv.Optional(CONF_LOG_UNKNOWN, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Pull in the Arduino-ESP32 SPI bundled library (the SoC's hardware SPI
    # driver, not the ESPHome `spi` component).
    cg.add_library("SPI", None)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_pins(
        config[CONF_MOSI_PIN],
        config[CONF_MISO_PIN],
        config[CONF_CLK_PIN],
        config[CONF_CS_PIN],
        config[CONF_GDO0_PIN],
        config.get(CONF_GDO2_PIN, -1),
    ))
    cg.add(var.set_frequency_hz(config[CONF_FREQUENCY]))
    cg.add(var.set_log_unknown(config[CONF_LOG_UNKNOWN]))
