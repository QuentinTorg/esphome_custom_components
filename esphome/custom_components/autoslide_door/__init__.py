import esphome.codegen as cg
from esphome.components import button, number, select, switch, text_sensor, uart
import esphome.config_validation as cv

# FIX: Renamed UNIT_OF_MEASUREMENT_SECOND to the current constant UNIT_SECOND
from esphome.const import (
    CONF_ICON,
    CONF_ID,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_NAME,
    CONF_OPTIONS,
    CONF_STEP,
    CONF_UART_ID,
    CONF_UNIT_OF_MEASUREMENT,
    UNIT_SECOND,
)

# --- NAMESPACE AND COMPONENT CLASSES ---

# Namespace for custom components
autoslide_door_ns = cg.esphome_ns.namespace("autoslide_door")

# The main custom component class inherits from cg.Component and the UART device helper
# It is critical that this class inherits from uart.UARTDevice to use its methods.
AutoslideDoor = autoslide_door_ns.class_("AutoslideDoor", cg.Component, uart.UARTDevice)

# Custom classes for the specific entity types
AutoslideTriggerButton = autoslide_door_ns.class_(
    "AutoslideTriggerButton", button.Button
)
AutoslideModeSelect = autoslide_door_ns.class_("AutoslideModeSelect", select.Select)
AutoslideOpenSpeedSwitch = autoslide_door_ns.class_(
    "AutoslideOpenSpeedSwitch", switch.Switch
)
AutoslideSecurePetSwitch = autoslide_door_ns.class_(
    "AutoslideSecurePetSwitch", switch.Switch
)
AutoslideOpenHoldNumber = autoslide_door_ns.class_(
    "AutoslideOpenHoldNumber", number.Number
)
AutoslideOpenForceNumber = autoslide_door_ns.class_(
    "AutoslideOpenForceNumber", number.Number
)
AutoslideCloseForceNumber = autoslide_door_ns.class_(
    "AutoslideCloseForceNumber", number.Number
)
AutoslideCloseEndForceNumber = autoslide_door_ns.class_(
    "AutoslideCloseEndForceNumber", number.Number
)
AutoslideMotionStateSensor = autoslide_door_ns.class_(
    "AutoslideMotionStateSensor", text_sensor.TextSensor
)
AutoslideLockStateSensor = autoslide_door_ns.class_(
    "AutoslideLockStateSensor", text_sensor.TextSensor
)


# --- CONFIGURATION SCHEMAS ---

# Schema for the Button (Master Trigger)
TRIGGER_BUTTON_SCHEMA = button.button_schema(AutoslideTriggerButton)

# Schema for the Select (Door Mode)
MODE_SELECT_OPTIONS = {
    "Auto": 0,
    "Stack": 1,
    "Lock": 2,
    "Pet": 3,
}
MODE_SELECT_SCHEMA = select.select_schema(
    AutoslideModeSelect,
    # FIX: Removed 'options=' keyword argument from select.select_schema.
).extend(
    {
        # FIX: Explicitly add CONF_OPTIONS validation for the Select component.
        # This allows the user to define options in YAML, which are then passed to register_select.
        cv.Required(CONF_OPTIONS): cv.All(
            cv.ensure_list(cv.string),
            # FIX: Ensure there are exactly 4 options. This is crucial since they map to fixed device modes 0-3.
            cv.Length(min=4, max=4),
            # FIX: Removed the unsupported 'cv.check_valid_options' call.
        ),
    }
)

# Schemas for Switches (Open Speed, Secure Pet)
OPEN_SPEED_SWITCH_SCHEMA = switch.switch_schema(AutoslideOpenSpeedSwitch)
SECURE_PET_SWITCH_SCHEMA = switch.switch_schema(AutoslideSecurePetSwitch)

# Schemas for Numbers (Open Hold, Forces)
OPEN_HOLD_NUMBER_SCHEMA = number.number_schema(
    AutoslideOpenHoldNumber,
    # FIX: Removed min_value, max_value, and step from here. They will be added
    # to the schema dictionary definition below to comply with newer ESPHome style.
    unit_of_measurement=UNIT_SECOND,
)
FORCE_NUMBER_SCHEMA = number.number_schema(
    # Note: Using the same class for all force number entities
    AutoslideOpenForceNumber,
    # FIX: Removed min_value, max_value, and step from here.
)

# Schemas for Text Sensors (Motion State, Lock State)
MOTION_STATE_SENSOR_SCHEMA = text_sensor.text_sensor_schema(AutoslideMotionStateSensor)
LOCK_STATE_SENSOR_SCHEMA = text_sensor.text_sensor_schema(AutoslideLockStateSensor)


# --- MAIN COMPONENT SCHEMA ---

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AutoslideDoor),
            # FIX applied previously: Relax the type check for uart_id to the base UARTComponent.
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            # Entity definitions
            cv.Required("trigger_button"): TRIGGER_BUTTON_SCHEMA,
            cv.Required("mode_select"): MODE_SELECT_SCHEMA,
            cv.Required("open_speed_switch"): OPEN_SPEED_SWITCH_SCHEMA,
            cv.Required("secure_pet_switch"): SECURE_PET_SWITCH_SCHEMA,
            # Numbers: Adding constraints back into the required schema fields
            cv.Required("open_hold_number"): OPEN_HOLD_NUMBER_SCHEMA.extend(
                {
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                    cv.Optional(CONF_MAX_VALUE, default=25): cv.float_,
                    cv.Optional(CONF_STEP, default=1): cv.float_,
                }
            ),
            cv.Required("open_open_force_number"): FORCE_NUMBER_SCHEMA.extend(
                {
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                    cv.Optional(CONF_MAX_VALUE, default=7): cv.float_,
                    cv.Optional(CONF_STEP, default=1): cv.float_,
                }
            ),
            cv.Required("close_force_number"): FORCE_NUMBER_SCHEMA.extend(
                {
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                    cv.Optional(CONF_MAX_VALUE, default=7): cv.float_,
                    cv.Optional(CONF_STEP, default=1): cv.float_,
                }
            ),
            cv.Required("close_end_force_number"): FORCE_NUMBER_SCHEMA.extend(
                {
                    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                    cv.Optional(CONF_MAX_VALUE, default=7): cv.float_,
                    cv.Optional(CONF_STEP, default=1): cv.float_,
                }
            ),
            cv.Required("motion_state_sensor"): MOTION_STATE_SENSOR_SCHEMA,
            cv.Required("lock_state_sensor"): LOCK_STATE_SENSOR_SCHEMA,
            # Time between status requests (e.g., to request current settings 'd:0')
            cv.Optional("update_interval", default="5s"): cv.update_interval,
            # Time between status updates from the Autoslide door (unprompted 'AT+UPSEND')
            cv.Optional(
                "status_timeout", default="30s"
            ): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


# --- CODE GENERATION ---


def to_code(config):
    # 1. Retrieve the UART component ID
    # FIX: Changed cg.get_component to cg.get_variable, which is the correct way
    # to retrieve the C++ pointer for another component's object in modern ESPHome.
    paren = yield cg.get_variable(config[CONF_UART_ID])

    # 2. Instantiate the main custom component
    var = cg.new_Pvariable(config[CONF_ID], paren)
    yield cg.register_component(var, config)

    # 3. Handle entity registrations

    # Button
    if "trigger_button" in config:
        conf = config["trigger_button"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield button.register_button(sens, conf)
        # FIX: Removed deprecated cg.setup_component. Button registration typically handles this internally.
        # cg.add(cg.setup_component(sens, conf))
        cg.add(var.set_trigger_button(sens))

    # Select
    if "mode_select" in config:
        conf = config["mode_select"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        # Use the options provided by the user in the configuration (conf[CONF_OPTIONS])
        options_list = conf[CONF_OPTIONS]
        yield select.register_select(sens, conf, options=options_list)
        cg.add(var.set_mode_select(sens))

        # FIX: Use cg.map(...) as a universal fall-back to generate the C++ map expression
        # when specific helpers like cg.std_map are missing in the dev environment.
        cg.add(sens.set_value_map(cg.map(MODE_SELECT_OPTIONS)))

    # Switches
    if "open_speed_switch" in config:
        conf = config["open_speed_switch"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield switch.register_switch(sens, conf)
        cg.add(var.set_open_speed_switch(sens))

    if "secure_pet_switch" in config:
        conf = config["secure_pet_switch"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield switch.register_switch(sens, conf)
        cg.add(var.set_secure_pet_switch(sens))

    # Numbers (The to_code part is correct as it pulls the required CONF_MIN/MAX_VALUE keys from the dictionary)
    if "open_hold_number" in config:
        conf = config["open_hold_number"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        # The arguments are now correctly retrieved from the dictionary generated by the CONFIG_SCHEMA extend
        yield number.register_number(
            sens,
            conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE],
            step=conf[CONF_STEP],
        )
        cg.add(var.set_open_hold_number(sens))

    if "open_open_force_number" in config:
        conf = config["open_open_force_number"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield number.register_number(
            sens,
            conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE],
            step=conf[CONF_STEP],
        )
        cg.add(var.set_open_open_force_number(sens))

    if "close_force_number" in config:
        conf = config["close_force_number"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield number.register_number(
            sens,
            conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE],
            step=conf[CONF_STEP],
        )
        cg.add(var.set_close_force_number(sens))

    if "close_end_force_number" in config:
        conf = config["close_end_force_number"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield number.register_number(
            sens,
            conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE],
            step=conf[CONF_STEP],
        )
        cg.add(var.set_close_end_force_number(sens))

    # Text Sensors
    if "motion_state_sensor" in config:
        conf = config["motion_state_sensor"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield text_sensor.register_text_sensor(sens, conf)
        cg.add(var.set_motion_state_sensor(sens))

    if "lock_state_sensor" in config:
        conf = config["lock_state_sensor"]
        sens = cg.new_Pvariable(conf[CONF_ID])
        yield text_sensor.register_text_sensor(sens, conf)
        cg.add(var.set_lock_state_sensor(sens))
