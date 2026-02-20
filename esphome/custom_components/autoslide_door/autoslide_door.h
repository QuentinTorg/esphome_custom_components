#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#include <optional>
#include <cstdint>

namespace esphome {
namespace autoslide_door {

static const char *const TAG = "autoslide_door";

// --- Constants & Enums based on Autoslide Programmer's Guide ---

enum class AutoslideTrigger
{
  MASTER = 0,
  INDOOR = 1,
  PET = 2,
  STACK = 3,
};

// Key 'a': Door Mode
enum class AutoslideMode
{
  UNKNOWN = -1,
  AUTO = 0,
  STACK = 1,
  LOCK = 2,
  PET = 3,
};

// Key 'e': Open Speed (Slow/Fast)
enum class AutoslideOpenSpeed
{
  FAST = 0, // Maps to Switch OFF
  SLOW = 1, // Maps to Switch ON (Default behavior)
};

// Key 'g': Secure Pet Mode (Pet On/Pet Off)
enum class AutoslideSecurePet
{
  ON = 0,  // Maps to Switch OFF
  OFF = 1, // Maps to Switch ON (Default behavior)
};

// Key 'm': Motion State (Read-only)
enum class AutoslideMotionState
{
  STOPPED = 0,
  OPENING = 1,
  CLOSING = 2,
};

// Key 'c': Lock State (Read-only)
enum class AutoslideLockedState
{
  UNLOCKED = 0,
  LOCKED = 1,
};

// Key identifiers (backed by the protocol characters)
enum class AutoslideKey : char
{
  NONE            = 0,   // null characer
  MODE            = 'a',
  LOCK_STATE      = 'c', // read-only
  OPEN_SPEED      = 'e',
  SECURE_PET      = 'g',
  HOLD_TIME       = 'j',
  OPEN_FORCE      = 'C',
  CLOSE_FORCE     = 'z',
  CLOSE_END_FORCE = 'A',
  MOTION_STATE    = 'm', // read-only
  MOTION_TRIGGER  = 'n', // read-only
  TRIGGER         = 'b', // action
  REQUEST_ALL     = 'd', // action (readback)
  RESULT          = 'r', // meta
};

// --- State Struct ---

struct AutoslideState
{
  AutoslideMode        door_mode{AutoslideMode::LOCK};
  AutoslideOpenSpeed   open_speed{AutoslideOpenSpeed::FAST};
  AutoslideSecurePet   secure_pet{AutoslideSecurePet::ON};
  uint8_t              open_hold_duration{0}; // 'j' (0-25)
  uint8_t              open_force{0};         // 'C' (0-7)
  uint8_t              close_force{0};        // 'z' (0-7)
  uint8_t              close_end_force{0};    // 'A' (0-7)
  AutoslideMotionState motion_state{AutoslideMotionState::STOPPED};
  AutoslideLockedState lock_state{AutoslideLockedState::LOCKED};
  uint8_t              motion_trigger{0};     // 'n' (raw)
  bool                 connected{false};
};

struct InflightUpdate {
    AutoslideKey key{AutoslideKey::NONE};
    int value{-1};
    uint32_t sent_time_ms{0};
};

// Free utility functions for manipulating the types
std::string mode_to_string(AutoslideMode mode);
std::string motion_state_to_string(AutoslideMotionState state);
bool speed_to_bool(const AutoslideOpenSpeed speed);
AutoslideOpenSpeed bool_to_speed(const bool speed_bool);
bool secure_pet_to_bool(const AutoslideSecurePet pet);
AutoslideSecurePet bool_to_secure_pet(const bool pet_bool);

// --- Main Component Class ---

class AutoslideDoor : public Component, public uart::UARTDevice
{
 public:
  // Component lifecycle methods
  float get_setup_priority() const override;
  void dump_config() override;
  void setup() override;
  void loop() override;

  // High-level requests (do not write UART directly)
  template <typename ValueT>
  void request_set_key_value(const AutoslideKey key, const ValueT value)
  {
      for (auto& [possible_key, request_value] : open_requests_)
      {
          if (possible_key == key)
          {
              request_value = static_cast<int>(value);
              return;
          }
      }

      ESP_LOGE(TAG, "Failed to queue key value send for key: %c, value: %d", static_cast<char>(key), static_cast<int>(value));
  }

  // publish current state to wifi devices/sensors
  bool publish_current_state(const bool full_publish = false);

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
  void set_connected_sensor(binary_sensor::BinarySensor *sensor);

  // temporary debug sensor
  void set_debug_uart_sensor(text_sensor::TextSensor *sensor) { debug_uart_sensor_ = sensor; }

 protected:
  // Internal: send a single AT+UPDATE or action; called only from loop reconcile
  bool send_update_command(const AutoslideKey key, const int value);
  void send_upsend_reply();
  bool send_next_update();

  // handle incoming commands
  void handle_incoming_command(const std::string &command);

  // parse helpers
  void parse_kv_payload(const char *payload, const size_t len);

  // update internal state based on new key/value params
  void update_state(const AutoslideKey key, const int value);

  // --- Internal Data ---

  // Current state and caches
  AutoslideState state_{};

  // array serves as a map, char value casted to int is the location in the array
  // ordering is also command send priority
  std::array<std::pair<AutoslideKey, std::optional<int>>, 9> open_requests_{{
      {AutoslideKey::TRIGGER, {}},
      {AutoslideKey::MODE, {}},
      {AutoslideKey::OPEN_SPEED, {}},
      {AutoslideKey::SECURE_PET, {}},
      {AutoslideKey::HOLD_TIME, {}},
      {AutoslideKey::OPEN_FORCE, {}},
      {AutoslideKey::CLOSE_FORCE, {}},
      {AutoslideKey::CLOSE_END_FORCE, {}},
      {AutoslideKey::REQUEST_ALL, {}},
  }};

  // Serial parsing
  std::string receive_buffer_;

  // TX bookkeeping
  std::optional<InflightUpdate> inflight_update_{};

  // Connection health
  uint32_t last_rx_time_ms_{0};   // last time we received any AT frame
  uint32_t last_poll_time_ms_{0}; // last time we sent a periodic poll
  bool first_poll_complete_{false}; // used to initiate immediate first poll

  // when is it okay to send commands even if door never sent first command
  // stamp set to 0 when first command arrives
  uint32_t startup_timeout_stamp_ms{25000};

  bool queued_upsend_reply_{false}; // must reply to every upsend from door

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
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  text_sensor::TextSensor *debug_uart_sensor_{nullptr};
};

// --- Custom Control Implementations ---

class AutoslideModeSelect : public select::Select
{
 public:
  AutoslideModeSelect() = default;
  explicit AutoslideModeSelect(AutoslideDoor *parent) : parent_(parent) {}
  void set_parent(AutoslideDoor *parent) { parent_ = parent; }
  void control(const std::string &value) override;

 protected:
  AutoslideDoor *parent_{nullptr};
};

class AutoslideSettingNumber : public number::Number
{
 public:
  AutoslideSettingNumber() = default;
  AutoslideSettingNumber(AutoslideDoor *parent, const AutoslideKey key) : parent_(parent) , key_(key) { }
  void set_parent(AutoslideDoor *parent) { parent_ = parent; }
  void set_key(const char key) { key_ = static_cast<AutoslideKey>(key); }
  void control(float value) override;

 protected:
  AutoslideDoor *parent_{nullptr};
  AutoslideKey key_{AutoslideKey::NONE};
};

class AutoslideOnOffSwitch : public switch_::Switch
{
 public:
  AutoslideOnOffSwitch() = default;
  AutoslideOnOffSwitch(AutoslideDoor *parent, const AutoslideKey key) : parent_(parent), key_(key) {}
  void set_parent(AutoslideDoor *parent) { parent_ = parent; }
  void set_key(const char key) { key_ = static_cast<AutoslideKey>(key); }
  void write_state(bool value) override;

 protected:
  AutoslideDoor *parent_{nullptr};
  AutoslideKey key_{AutoslideKey::NONE};
};

class AutoslideOpenButton : public button::Button
{
 public:
  AutoslideOpenButton() = default;
  explicit AutoslideOpenButton(AutoslideDoor *parent) : parent_(parent) {}
  void set_parent(AutoslideDoor *parent) { parent_ = parent; }

 protected:
  void press_action() override;

  AutoslideDoor *parent_{nullptr};
};


} // namespace autoslide_door
} // namespace esphome
