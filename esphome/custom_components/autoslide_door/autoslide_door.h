#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"
#include <vector>

namespace esphome {
namespace autoslide_door {

static const char *const TAG = "autoslide_door";
static const uint8_t END_CHAR = 0x1B; // Escape character (\e)

// --- Custom Entity Classes ---

// Base class for all controllable entities to access the parent component
class AutoslideDoorComponentBase {
 public:
  void set_parent(AutoslideDoor *parent) { this->parent_ = parent; }

 protected:
  AutoslideDoor *parent_{nullptr};
};

class AutoslideTriggerButton : public button::Button, public AutoslideDoorComponentBase {
 public:
  void press_action() override;
};

class AutoslideModeSelect : public select::Select, public AutoslideDoorComponentBase {
 public:
  void set_value_map(const std::map<std::string, int> &value_map) { this->value_map_ = value_map; }
  void control(const std::string &value) override;

 protected:
  std::map<std::string, int> value_map_;
};

class AutoslideOpenSpeedSwitch : public switch_::Switch, public AutoslideDoorComponentBase {
 public:
  void write_state(bool state) override;
};

class AutoslideSecurePetSwitch : public switch_::Switch, public AutoslideDoorComponentBase {
 public:
  void write_state(bool state) override;
};

class AutoslideOpenHoldNumber : public number::Number, public AutoslideDoorComponentBase {
 public:
  void control(float value) override;
};

class AutoslideOpenForceNumber : public number::Number, public AutoslideDoorComponentBase {
 public:
  void control(float value) override;
};

class AutoslideCloseForceNumber : public number::Number, public AutoslideDoorComponentBase {
 public:
  void control(float value) override;
};

class AutoslideCloseEndForceNumber : public number::Number, public AutoslideDoorComponentBase {
 public:
  void control(float value) override;
};

class AutoslideMotionStateSensor : public text_sensor::TextSensor, public AutoslideDoorComponentBase {};
class AutoslideLockStateSensor : public text_sensor::TextSensor, public AutoslideDoorComponentBase {};


// --- Main Custom Component ---

class AutoslideDoor : public Component, public uart::UARTDevice {
 public:
  // --- Setter methods for entities ---
  void set_trigger_button(AutoslideTriggerButton *btn) { this->trigger_button_ = btn; }
  void set_mode_select(AutoslideModeSelect *sel) { this->mode_select_ = sel; }
  void set_open_speed_switch(AutoslideOpenSpeedSwitch *sw) { this->open_speed_switch_ = sw; }
  void set_secure_pet_switch(AutoslideSecurePetSwitch *sw) { this->secure_pet_switch_ = sw; }
  void set_open_hold_number(AutoslideOpenHoldNumber *num) { this->open_hold_number_ = num; }
  void set_open_open_force_number(AutoslideOpenForceNumber *num) { this->open_open_force_number_ = num; }
  void set_close_force_number(AutoslideCloseForceNumber *num) { this->close_force_number_ = num; }
  void set_close_end_force_number(AutoslideCloseEndForceNumber *num) { this->close_end_force_number_ = num; }
  void set_motion_state_sensor(AutoslideMotionStateSensor *sen) { this->motion_state_sensor_ = sen; }
  void set_lock_state_sensor(AutoslideLockStateSensor *sen) { this->lock_state_sensor_ = sen; }

  // --- ESPHome Component Overrides ---
  void setup() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // --- Autoslide Command Methods ---
  void send_command(const std::string &command, uint32_t wait_ms = 50);
  void request_status();
  void parse_status(const std::string &payload);

  // --- Entity Callbacks ---
  void trigger_master();
  void set_door_mode(int mode);
  void set_open_speed(bool state);
  void set_secure_pet(bool state);
  void set_open_hold_time(float value);
  void set_open_force(float value);
  void set_close_force(float value);
  void set_close_end_force(float value);

 protected:
  // --- Internal state and buffers ---
  std::string receive_buffer_;
  uint32_t last_command_time_{0};
  uint32_t last_status_update_time_{0};
  bool waiting_for_result_{false};

  // --- Pointers to all entities ---
  AutoslideTriggerButton *trigger_button_{nullptr};
  AutoslideModeSelect *mode_select_{nullptr};
  AutoslideOpenSpeedSwitch *open_speed_switch_{nullptr};
  AutoslideSecurePetSwitch *secure_pet_switch_{nullptr};
  AutoslideOpenHoldNumber *open_hold_number_{nullptr};
  AutoslideOpenForceNumber *open_open_force_number_{nullptr};
  AutoslideCloseForceNumber *close_force_number_{nullptr};
  AutoslideCloseEndForceNumber *close_end_force_number_{nullptr};
  AutoslideMotionStateSensor *motion_state_sensor_{nullptr};
  AutoslideLockStateSensor *lock_state_sensor_{nullptr};

  // Utility function for parsing key-value pairs
  std::map<std::string, std::string> parse_key_values(const std::string &payload);
};

} // namespace autoslide_door
} // namespace esphome
