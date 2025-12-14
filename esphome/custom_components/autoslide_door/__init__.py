import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import uart, select, switch, number, text_sensor, button, binary_sensor
from esphome.const import (
    CONF_ID,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["select", "switch", "number", "text_sensor", "button", "binary_sensor"]

autoslide_ns = cg.esphome_ns.namespace("autoslide_door")
AutoslideDoor = autoslide_ns.class_(
    "AutoslideDoor", cg.Component, uart.UARTDevice
)

# --- Configuration keys ---

CONF_MODE_SELECT = "mode_select"
CONF_OPEN_SPEED_SWITCH = "open_speed_switch"
CONF_SECURE_PET_SWITCH = "secure_pet_switch"

CONF_OPEN_HOLD_NUMBER = "open_hold_duration"
CONF_OPEN_FORCE_NUMBER = "open_force"
CONF_CLOSE_FORCE_NUMBER = "close_force"
CONF_CLOSE_END_FORCE_NUMBER = "close_end_force"

CONF_MOTION_STATE_SENSOR = "motion_state"
CONF_LOCK_STATE_SENSOR = "lock_state"

CONF_OPEN_BUTTON = "open_button"
CONF_BUSY_SENSOR = "busy"

# --- Classes for entities we construct ---
AutoslideModeSelect = autoslide_ns.class_("AutoslideModeSelect", select.Select)
AutoslideOnOffSwitch = autoslide_ns.class_("AutoslideOnOffSwitch", switch.Switch)
AutoslideSettingNumber = autoslide_ns.class_("AutoslideSettingNumber", number.Number)
AutoslideOpenButton = autoslide_ns.class_("AutoslideOpenButton", button.Button)

# --- Schema ---

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AutoslideDoor),

            cv.Optional(CONF_MODE_SELECT): select.select_schema(AutoslideModeSelect),

            cv.Optional(CONF_OPEN_SPEED_SWITCH): switch.switch_schema(AutoslideOnOffSwitch),

            cv.Optional(CONF_SECURE_PET_SWITCH): switch.switch_schema(AutoslideOnOffSwitch),

            cv.Optional(CONF_OPEN_HOLD_NUMBER): number.number_schema(AutoslideSettingNumber),

            cv.Optional(CONF_OPEN_FORCE_NUMBER): number.number_schema(AutoslideSettingNumber),

            cv.Optional(CONF_CLOSE_FORCE_NUMBER): number.number_schema(AutoslideSettingNumber),

            cv.Optional(CONF_CLOSE_END_FORCE_NUMBER): number.number_schema(AutoslideSettingNumber),

            cv.Optional(CONF_MOTION_STATE_SENSOR): text_sensor.text_sensor_schema(),

            cv.Optional(CONF_LOCK_STATE_SENSOR): text_sensor.text_sensor_schema(),

            cv.Optional(CONF_OPEN_BUTTON): button.button_schema(AutoslideOpenButton),

            cv.Optional(CONF_BUSY_SENSOR): binary_sensor.binary_sensor_schema(),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

# --- Code Generation ---

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # --- Mode Select ---
    if CONF_MODE_SELECT in config:
        sel = await select.new_select(
            config[CONF_MODE_SELECT],
            options=["Auto", "Stack", "Lock", "Pet"],
        )
        cg.add(sel.set_parent(var))
        cg.add(var.set_mode_select(sel))

    # --- Open Speed Switch ('e') ---
    if CONF_OPEN_SPEED_SWITCH in config:
        sw = await switch.new_switch(config[CONF_OPEN_SPEED_SWITCH])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_key(ord('e')))
        cg.add(var.set_open_speed_switch(sw))

    # --- Secure Pet Switch ('g') ---
    if CONF_SECURE_PET_SWITCH in config:
        sw = await switch.new_switch(config[CONF_SECURE_PET_SWITCH])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_key(ord('g')))
        cg.add(var.set_secure_pet_switch(sw))

    # --- Numbers ---
    if CONF_OPEN_HOLD_NUMBER in config:
        num = await number.new_number(
            config[CONF_OPEN_HOLD_NUMBER],
            min_value=0,
            max_value=25,
            step=1,
        )
        cg.add(num.set_parent(var))
        cg.add(num.set_key(ord('j')))
        cg.add(var.set_open_hold_number(num))

    if CONF_OPEN_FORCE_NUMBER in config:
        num = await number.new_number(
            config[CONF_OPEN_FORCE_NUMBER],
            min_value=0,
            max_value=7,
            step=1,
        )
        cg.add(num.set_parent(var))
        cg.add(num.set_key(ord('C')))
        cg.add(var.set_open_force_number(num))

    if CONF_CLOSE_FORCE_NUMBER in config:
        num = await number.new_number(
            config[CONF_CLOSE_FORCE_NUMBER],
            min_value=0,
            max_value=7,
            step=1,
        )
        cg.add(num.set_parent(var))
        cg.add(num.set_key(ord('z')))
        cg.add(var.set_close_force_number(num))

    if CONF_CLOSE_END_FORCE_NUMBER in config:
        num = await number.new_number(
            config[CONF_CLOSE_END_FORCE_NUMBER],
            min_value=0,
            max_value=7,
            step=1,
        )
        cg.add(num.set_parent(var))
        cg.add(num.set_key(ord('A')))
        cg.add(var.set_close_end_force_number(num))

    # --- Text Sensors ---
    if CONF_MOTION_STATE_SENSOR in config:
        sensor = await text_sensor.new_text_sensor(
            config[CONF_MOTION_STATE_SENSOR]
        )
        cg.add(var.set_motion_state_sensor(sensor))

    if CONF_LOCK_STATE_SENSOR in config:
        sensor = await text_sensor.new_text_sensor(
            config[CONF_LOCK_STATE_SENSOR]
        )
        cg.add(var.set_lock_state_sensor(sensor))

    # --- Open Button ---
    if CONF_OPEN_BUTTON in config:
        btn = await button.new_button(config[CONF_OPEN_BUTTON])
        cg.add(btn.set_parent(var))
        cg.add(var.set_open_button(btn))

    # --- Busy Sensor ---
    if CONF_BUSY_SENSOR in config:
        busy = await binary_sensor.new_binary_sensor(
            config[CONF_BUSY_SENSOR]
        )
        cg.add(var.set_busy_sensor(busy))