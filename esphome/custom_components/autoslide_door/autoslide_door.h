#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome
{
namespace autoslide_door
{

// --- Constants & Enums based on Autoslide Programmer's Guide ---

// Key 'b'
static const int TRIGGER_MASTER = 0;

// Key 'a': Door Mode
enum AutoslideMode
{
  MODE_AUTO = 0,
  MODE_STACK = 1,
  MODE_LOCK = 2,
  MODE_PET = 3,
};

// Key 'e': Open Speed (Slow/Fast)
enum AutoslideOpenSpeed
{
  OPEN_SPEED_FAST = 0, // Maps to Switch OFF
  OPEN_SPEED_SLOW = 1, // Maps to Switch ON (Default behavior)
};

// Key 'g': Secure Pet Mode (Pet On/Pet Off)
enum AutoslideSecurePet
{
  SECURE_PET_ON = 0,  // Maps to Switch OFF
  SECURE_PET_OFF = 1, // Maps to Switch ON (Default behavior)
};

// Key 'm': Motion State (Read-only)
enum AutoslideMotionState
{
  MOTION_STOPPED = 0,
  MOTION_OPENING = 1,
  MOTION_CLOSING = 2,
};

// Key 'c': Lock State (Read-only)
enum AutoslideLockedState
{
  STATE_UNLOCKED = 0,
  STATE_LOCKED = 1,
};

// --- State Struct ---

struct AutoslideState
{
  AutoslideMode door_mode{MODE_LOCK};
  AutoslideOpenSpeed open_speed{OPEN_SPEED_FAST};
  AutoslideSecurePet secure_pet{SECURE_PET_ON};
  uint8_t open_hold_duration{0}; // Key 'j' (0-25)
  uint8_t open_force{0};         // Key 'C' (0-7)
  uint8_t close_force{0};        // Key 'z' (0-7)
  uint8_t close_end_force{0};    // Key 'A' (0-7)
  AutoslideMotionState motion_state{MOTION_STOPPED};
  AutoslideLockedState lock_state{STATE_LOCKED};
  uint8_t motion_trigger{0};     // Key 'n' (Raw trigger value)
};

// Forward declaration of the main component class
class AutoslideDoor;

// --- Custom Control Implementations ---

// Custom Select class for Door Mode ('a')
class AutoslideModeSelect : public select::Select
{
 public:
  AutoslideModeSelect(AutoslideDoor *parent) : parent_(parent) {}
  void control(const std::string &value) override;
 protected:
  AutoslideDoor *parent_;
};

// Custom Number class for Open Hold, Forces ('j', 'C', 'Z', 'A')
class AutoslideSettingNumber : public number::Number
{
 public:
  AutoslideSettingNumber(AutoslideDoor *parent, char key) : parent_(parent), key_(key) {}
  void control(float value) override;
 protected:
  AutoslideDoor *parent_;
  char key_;
};

// Custom Switch class for Open Speed and Secure Pet ('e', 'g')
class AutoslideOnOffSwitch : public switch_::Switch
{
 public:
  AutoslideOnOffSwitch(AutoslideDoor *parent, char key) : parent_(parent), key_(key) {}
  void write_state(bool value) override;
 protected:
  AutoslideDoor *parent_;
  char key_;
};

class AutoslideOpenButton : public button::Button
{
 public:
  explicit AutoslideOpenButton(AutoslideDoor *parent) : parent_(parent) {}

 protected:
  void press_action() override;

  AutoslideDoor *parent_;
};

// --- Main Component Class ---

class AutoslideDoor : public Component, public uart::UARTDevice
{
 public:
  // Component lifecycle methods
  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

  // Custom actions
  void trigger_open();

  // Protocol methods
  bool send_update_command(char key, int value);

  // Utility functions
  std::string mode_to_string(AutoslideMode mode) const;
  std::string motion_state_to_string(AutoslideMotionState state) const;
  bool speed_to_bool(AutoslideOpenSpeed speed) const;
  bool secure_pet_to_bool(AutoslideSecurePet pet) const;

  // Private helper functions
  void request_all_settings();
  void handle_incoming_command(const std::string &command);
  void handle_result_command(const std::string &payload);
  void handle_upsend_command(const std::string &payload);
  void send_upsend_reply();
  void update_state(char key, int value);
  void publish_current_state();

  // ESPHome Configuration Setter Methods
  void set_mode_select(select::Select *select);
  void set_open_speed_switch(switch_::Switch *sw);
  void set_secure_pet_switch(switch_::Switch *sw);
  void set_open_hold_number(number::Number *number);
  void set_open_force_number(number::Number *number);
  void set_close_force_number(number::Number *number);
  void set_close_end_force_number(number::Number *number);
  void set_motion_state_sensor(text_sensor::TextSensor *sensor);
  void set_lock_state_sensor(text_sensor::TextSensor *sensor);
  void set_open_button(button::Button *button);
  void set_busy_sensor(binary_sensor::BinarySensor *sensor);

 protected:
  AutoslideState state_{};
  std::string receive_buffer_;
  bool awaiting_result_from_update_{false};
  uint32_t last_command_sent_time_ms_{0};

  // Pointers to the ESPHome entities defined in YAML
  select::Select *mode_select_{nullptr};
  switch_::Switch *open_speed_switch_{nullptr};
  switch_::Switch *secure_pet_switch_{nullptr};
  number::Number *open_hold_number_{nullptr};
  number::Number *open_force_number_{nullptr};
  number::Number *close_force_number_{nullptr};
  number::Number *close_end_force_number_{nullptr};
  text_sensor::TextSensor *motion_state_sensor_{nullptr};
  text_sensor::TextSensor *lock_state_sensor_{nullptr};
  button::Button *open_button_{nullptr};
  binary_sensor::BinarySensor *busy_sensor_{nullptr};
};

} // namespace autoslide_door
} // namespace esphome
