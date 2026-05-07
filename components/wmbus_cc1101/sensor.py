"""Sensor platform for wmbus_cc1101.

Each ``- platform: wmbus_cc1101`` block registers a per-meter listener that
publishes ``total_water_m3`` and ``rssi`` whenever a matching telegram is
received.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_KEY,
    CONF_TYPE,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_WATER,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_DECIBEL_MILLIWATT,
)

from . import WMBusComponent, wmbus_cc1101_ns

DEPENDENCIES = ["wmbus_cc1101"]

CONF_HUB_ID = "wmbus_cc1101_id"
CONF_METER_ID = "meter_id"
CONF_TOTAL_WATER_M3 = "total_water_m3"
CONF_RSSI = "rssi"

UNIT_CUBIC_METER = "m³"

WMBusSensor = wmbus_cc1101_ns.class_("WMBusSensor", cg.Component)

METER_TYPES = {
    "apator162": wmbus_cc1101_ns.namespace("MeterType").enum("APATOR162"),
}


def _hex_key_32(value):
    """Validate a 16-byte AES key as 32 hex digits, allow 'NOKEY' or all-zeros."""
    if value is None:
        return ""
    if not isinstance(value, str):
        raise cv.Invalid("key must be a hex string")
    s = value.strip().replace(" ", "").replace(":", "")
    if s.lower() in ("", "nokey", "none"):
        return ""
    if len(s) != 32:
        raise cv.Invalid("key must be 32 hex characters (16 bytes)")
    try:
        int(s, 16)
    except ValueError as err:
        raise cv.Invalid("key contains non-hex characters") from err
    return s.lower()


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WMBusSensor),
        cv.GenerateID(CONF_HUB_ID): cv.use_id(WMBusComponent),
        cv.Required(CONF_METER_ID): cv.hex_uint32_t,
        cv.Optional(CONF_TYPE, default="apator162"): cv.enum(METER_TYPES, lower=True),
        cv.Optional(CONF_KEY, default=""): _hex_key_32,
        cv.Optional(CONF_TOTAL_WATER_M3): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_METER,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_WATER,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_RSSI): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_HUB_ID])
    cg.add(var.set_meter_id(config[CONF_METER_ID]))
    cg.add(var.set_meter_type(config[CONF_TYPE]))
    cg.add(var.set_key_hex(config[CONF_KEY]))

    if CONF_TOTAL_WATER_M3 in config:
        s = await sensor.new_sensor(config[CONF_TOTAL_WATER_M3])
        cg.add(var.set_total_water_sensor(s))

    if CONF_RSSI in config:
        s = await sensor.new_sensor(config[CONF_RSSI])
        cg.add(var.set_rssi_sensor(s))

    cg.add(hub.register_meter(var))
