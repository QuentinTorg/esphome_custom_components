#include "autoslide_door.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <sstream>

namespace esphome {
namespace autoslide_door {

static const char *const TAG = "autoslide_door";
static const uint32_t COMMAND_TIMEOUT_MS = 15000; // 15 seconds as per the guide

// --- Helper Functions for String Conversion (from .h) ---
//

std::string AutoslideDoor::mode_to_string(AutoslideMode mode) const
{
  switch (mode)
  {
    case MODE_AUTO:
      return "Auto";
    case MODE_STACK:
      return "Stack";
    case MODE_LOCK:
      return "Lock";
    case MODE_PET:
      return "Pet";
    default:
      return "Unknown";
  }
}

std::string AutoslideDoor::motion_state_to_string(AutoslideMotionState state) const
{
  switch (state)
  {
    case MOTION_STOPPED:
      return "Stopped";
    case MOTION_OPENING:
      return "Opening";
    case MOTION_CLOSING:
      return "Closing";
    default:
      return "Unknown";
  }
}

bool AutoslideDoor::speed_to_bool(AutoslideOpenSpeed speed) const
{
  // OPEN_SPEED_SLOW (1) is 'ON' for the switch, OPEN_SPEED_FAST (0) is 'OFF'
  return speed == OPEN_SPEED_SLOW;
}

bool AutoslideDoor::secure_pet_to_bool(AutoslideSecurePet pet) const
{
  // SECURE_PET_OFF (1) is 'ON' for the switch, SECURE_PET_ON (0) is 'OFF'
  return pet == SECURE_PET_OFF;
}

// --- AutoslideDoor Component Implementation ---

void AutoslideDoor::setup()
{
  ESP_LOGCONFIG(TAG, "Setting up Autoslide Door Component...");
  // Initialize state to default or unknown values
  state_ = {};

  // Request all current settings from the door right away
  request_all_settings();
}

float AutoslideDoor::get_setup_priority() const
{
  // UART setup should happen early
  return setup_priority::BUS;
}

void AutoslideDoor::dump_config()
{
    ESP_LOGCONFIG(TAG, "autoslide component");
    ESP_LOGCONFIG(TAG, "  awaiting_result_from_update: %s", awaiting_result_from_update_ ? "true" : "false");
    ESP_LOGCONFIG(TAG, "  last_command_sent_time_ms: %i", last_command_sent_time_ms_);

    ESP_LOGCONFIG(TAG, "  now:                       %i", esphome::millis());
    ESP_LOGCONFIG(TAG, "  state:");
    ESP_LOGCONFIG(TAG, "    mode: %s", mode_to_string(state_.door_mode));
    ESP_LOGCONFIG(TAG, "    open_speed %s", state_.open_speed == OPEN_SPEED_FAST ? "fast" : "slow");
    ESP_LOGCONFIG(TAG, "    secure_pet: %s", state_.secure_pet == SECURE_PET_ON ? "on" : "off");
    ESP_LOGCONFIG(TAG, "    open_hold_duration: %i", state_.open_hold_duration);
    ESP_LOGCONFIG(TAG, "    open_force: %i", state_.open_force);
    ESP_LOGCONFIG(TAG, "    close_force: %i", state_.close_force);
    ESP_LOGCONFIG(TAG, "    close_end_force: %i", state_.close_end_force);
    ESP_LOGCONFIG(TAG, "    motion_state: %s", motion_state_to_string(state_.motion_state));
    ESP_LOGCONFIG(TAG, "    lock_state: %s", state_.lock_state == STATE_LOCKED ? "locked" : "unlocked");
    ESP_LOGCONFIG(TAG, "    motion_trigger: %i", state_.motion_trigger);
}

void AutoslideDoor::loop()
{
  // 1. Read incoming UART data
  while (available())
  {
    char byte;
    read_byte(&byte);

    // The Autoslide protocol uses the escape character (0x1B or '\e') to terminate commands.
    if (byte == 0x1B)
    {
      // Complete command received. Process it.
      if (!receive_buffer_.empty())
      {
        handle_incoming_command(receive_buffer_);
      }
      // Clear buffer for the next command
      receive_buffer_.clear();
    }
    else
    {
      // Append byte to the receive buffer
      receive_buffer_ += byte;
    }
  }

  // 2. Handle Command Timeout
  // If we sent a command but haven't received a result in 15 seconds, we assume it failed
  // and clear the flag to allow new commands. This uses rollover-safe arithmetic.
  if (awaiting_result_from_update_)
  {
    uint32_t now = esphome::millis();
    if (now - last_command_sent_time_ms_ > COMMAND_TIMEOUT_MS)
    {
      ESP_LOGE(TAG, "Command timeout! Did not receive AT+RESULT within %u ms.", COMMAND_TIMEOUT_MS);
      awaiting_result_from_update_ = false;
    }
  }
}

void AutoslideDoor::trigger_open()
{
  // Key 'b' is for trigger. Value 0 is TRIGGER_MASTER (the main open command).
  // This command must follow the AT+UPDATE protocol to ensure command sequencing.
  if (send_update_command('b', TRIGGER_MASTER))
  {
    ESP_LOGD(TAG, "Sent Master Open Trigger (b:0)");
  }
}

// --- Utility Functions ---

bool AutoslideDoor::send_update_command(char key, int value)
{
  // Check if we are currently waiting for a result from a previous command
  if (awaiting_result_from_update_)
  {
    ESP_LOGW(TAG, "Cannot send command ('%c':%d). Waiting for AT+RESULT from previous command.", key, value);
    return false;
  }

  // Format the command string: AT+UPDATE,key:value\e
  std::string command = "AT+UPDATE,";
  command += key;
  command += ":";
  if (key == 'j' and value >= 0 and value <= 9)
  {
      // for j value only output must be 2 digits
      command += '0';
  }
  command += std::to_string(value);
  command += (char) 0x1B; // Escape character

  ESP_LOGD(TAG, "Sending command: %s", command.c_str());

  // Write to UART
  write_str(command.c_str());

  // Set flags to wait for the reply
  awaiting_result_from_update_ = true;
  last_command_sent_time_ms_ = esphome::millis();
  return true;
}

void AutoslideDoor::send_upsend_reply()
{
  // Required reply after receiving AT+UPSEND: AT+REPLY,r:1\e
  const std::string command = "AT+REPLY,r:1" + std::string(1, 0x1B);
  ESP_LOGD(TAG, "Sending UPSEND Reply: %s", command.c_str());
  write_str(command.c_str());
}

void AutoslideDoor::request_all_settings()
{
  // Key 'd' with value 0 requests all current settings: AT+UPDATE,d:0\e
  if (send_update_command('d', 0))
  {
    ESP_LOGI(TAG, "Requesting initial door settings (d:0)...");
  }
}

void AutoslideDoor::handle_incoming_command(const std::string &command)
{
  ESP_LOGV(TAG, "Received raw command: %s", command.c_str());

  // Check for the mandatory "AT+" prefix
  if (command.length() < 3 || command.substr(0, 3) != "AT+")
  {
    ESP_LOGE(TAG, "Invalid AT command prefix: %s", command.c_str());
    return;
  }

  // Find the command type (e.g., RESULT, UPSEND)
  size_t comma_pos = command.find(',');
  std::string command_type;
  std::string payload;

  if (comma_pos == std::string::npos)
  {
    // Command with no payload (e.g., AT+UPDATE without comma, unlikely but check basic structure)
    command_type = command.substr(3);
  }
  else
  {
    // Command with payload: AT+COMMAND,payload
    command_type = command.substr(3, comma_pos - 3);
    payload = command.substr(comma_pos + 1);
  }

  if (command_type == "RESULT")
  {
    handle_result_command(payload);
  }
  else if (command_type == "UPSEND")
  {
    handle_upsend_command(payload);
  }
  else if (command_type == "REPLY")
  {
    // The door is responding to our AT+REPLY. Nothing to do here.
    ESP_LOGV(TAG, "Received AT+REPLY from Autoslide. Acknowledged.");
  }
  else
  {
    ESP_LOGW(TAG, "Unknown AT command type received: %s", command_type.c_str());
  }
}

void AutoslideDoor::handle_result_command(const std::string &payload)
{
  // AT+RESULT is the response to AT+UPDATE. It may contain r:1/r:0 and/or updated settings.
  ESP_LOGD(TAG, "Received AT+RESULT with payload: %s", payload.c_str());

  // Processing the AT+RESULT means the update cycle is complete.
  awaiting_result_from_update_ = false;

  // Split payload by comma to get individual key-value pairs
  std::stringstream ss(payload);
  std::string key_value_pair;

  while (std::getline(ss, key_value_pair, ','))
  {
    // Each pair is key:value (e.g., "a:0" or "r:1")
    size_t colon_pos = key_value_pair.find(':');
    if (colon_pos == std::string::npos || key_value_pair.length() < 3)
    {
      ESP_LOGW(TAG, "Malformed key-value pair in AT+RESULT: %s", key_value_pair.c_str());
      continue;
    }

    char key = key_value_pair[0];
    std::string value_str = key_value_pair.substr(colon_pos + 1);
    int value = atoi(value_str.c_str());

    if (key == 'r')
    {
      // Status result (r:1 = success, r:0 = fail)
      if (value == 1)
      {
        ESP_LOGI(TAG, "Command acknowledged successfully.");
      }
      else
      {
        ESP_LOGE(TAG, "Command failed to execute (r:0).");
      }
    }
    else
    {
      // This is a setting or state update. Update the internal state.
      update_state(key, value);
    }
  }
  // Publish all updated entities to ESPHome only once after processing the full response
  publish_current_state();
}

void AutoslideDoor::handle_upsend_command(const std::string &payload)
{
  // AT+UPSEND is an unsolicited message with current status (motion, lock, etc.)
  ESP_LOGD(TAG, "Received AT+UPSEND (Status Update) with payload: %s", payload.c_str());

  // We must reply immediately to acknowledge the status update.
  send_upsend_reply();

  // Split payload and process key-value pairs
  std::stringstream ss(payload);
  std::string key_value_pair;

  while (std::getline(ss, key_value_pair, ','))
  {
    size_t colon_pos = key_value_pair.find(':');
    if (colon_pos == std::string::npos || key_value_pair.length() < 3)
    {
      ESP_LOGW(TAG, "Malformed key-value pair in AT+UPSEND: %s", key_value_pair.c_str());
      continue;
    }

    char key = key_value_pair[0];
    std::string value_str = key_value_pair.substr(colon_pos + 1);
    int value = atoi(value_str.c_str());

    // Update the internal state for each key/value pair
    update_state(key, value);
  }
  // Publish all updated entities to ESPHome only once after processing the full response
  publish_current_state();
}

void AutoslideDoor::update_state(char key, int value)
{
  // Process and update internal state based on the key
  switch (key)
  {
    // Writable Settings
    case 'a': state_.door_mode = (AutoslideMode) value; break;
    case 'e': state_.open_speed = (AutoslideOpenSpeed) value; break;
    case 'g': state_.secure_pet = (AutoslideSecurePet) value; break;
    case 'j': state_.open_hold_duration = (uint8_t) value; break;
    case 'C': state_.open_force = (uint8_t) value; break;
    case 'z': state_.close_force = (uint8_t) value; break;
    case 'A': state_.close_end_force = (uint8_t) value; break;

    // Read-only Status
    case 'm': state_.motion_state = (AutoslideMotionState) value; break;
    case 'c': state_.lock_state = (AutoslideLockedState) value; break;
    case 'n': state_.motion_trigger = (uint8_t) value; break; // Not exposed as an entity, just log
    default:
      if (key != 'r')
      {
        ESP_LOGW(TAG, "Received unknown key '%c' with value %d", key, value);
      }
      break;
  }
}

void AutoslideDoor::publish_current_state()
{
  ESP_LOGV(TAG, "Publishing current state to ESPHome entities...");

  // Mode Select
  if (mode_select_ != nullptr)
  {
    mode_select_->publish_state(mode_to_string(state_.door_mode));
  }
  // Open Speed Switch
  if (open_speed_switch_ != nullptr)
  {
    open_speed_switch_->publish_state(speed_to_bool(state_.open_speed));
  }
  // Secure Pet Switch
  if (secure_pet_switch_ != nullptr)
  {
    secure_pet_switch_->publish_state(secure_pet_to_bool(state_.secure_pet));
  }
  // Open Hold Number
  if (open_hold_number_ != nullptr)
  {
    open_hold_number_->publish_state(state_.open_hold_duration);
  }
  // Open Force Number
  if (open_force_number_ != nullptr)
  {
    open_force_number_->publish_state(state_.open_force);
  }
  // Close Force Number
  if (close_force_number_ != nullptr)
  {
    close_force_number_->publish_state(state_.close_force);
  }
  // Close End Force Number
  if (close_end_force_number_ != nullptr)
  {
    close_end_force_number_->publish_state(state_.close_end_force);
  }
  // Motion State Text Sensor
  if (motion_state_sensor_ != nullptr)
  {
    const char *motion_str;
    switch (state_.motion_state)
    {
      case MOTION_STOPPED: motion_str = "Stopped"; break;
      case MOTION_OPENING: motion_str = "Opening"; break;
      case MOTION_CLOSING: motion_str = "Closing"; break;
      default: motion_str = "Unknown"; break;
    }
    motion_state_sensor_->publish_state(motion_str);
  }
  // Lock State Text Sensor
  if (lock_state_sensor_ != nullptr)
  {
    const char *lock_str = (state_.lock_state == STATE_LOCKED) ? "Locked" : "Unlocked";
    lock_state_sensor_->publish_state(lock_str);
  }
}

// --- ESPHome Configuration Setter Methods ---

void AutoslideDoor::set_mode_select(select::Select *select) { mode_select_ = select; }
void AutoslideDoor::set_open_speed_switch(switch_::Switch *sw) { open_speed_switch_ = sw; }
void AutoslideDoor::set_secure_pet_switch(switch_::Switch *sw) { secure_pet_switch_ = sw; }
void AutoslideDoor::set_open_hold_number(number::Number *number) { open_hold_number_ = number; }
void AutoslideDoor::set_open_force_number(number::Number *number) { open_force_number_ = number; }
void AutoslideDoor::set_close_force_number(number::Number *number) { close_force_number_ = number; }
void AutoslideDoor::set_close_end_force_number(number::Number *number) { close_end_force_number_ = number; }
void AutoslideDoor::set_motion_state_sensor(text_sensor::TextSensor *sensor) { motion_state_sensor_ = sensor; }
void AutoslideDoor::set_lock_state_sensor(text_sensor::TextSensor *sensor) { lock_state_sensor_ = sensor; }

// --- Custom Entity Control Implementations ---

void AutoslideModeSelect::control(const std::string &value)
{
  int mode_value = -1;
  // Convert mode string back to its corresponding integer value
  if (value == "Auto")
  {
    mode_value = MODE_AUTO;
  }
  else if (value == "Stack")
  {
    mode_value = MODE_STACK;
  }
  else if (value == "Lock")
  {
    mode_value = MODE_LOCK;
  }
  else if (value == "Pet")
  {
    mode_value = MODE_PET;
  }

  if (mode_value != -1)
  {
    // Send the AT command: key 'a' for mode
    if (parent_->send_update_command('a', mode_value))
    {
      ESP_LOGI(TAG, "Sent mode command: %s. (converted to %s)", value, parent_->mode_to_string(mode_value));
    }
  }
  else
  {
    ESP_LOGE(TAG, "Invalid door mode selected: %s", value.c_str());
  }
}

void AutoslideSettingNumber::control(float value)
{
  int int_value = (int) roundf(value); // Ensure it's treated as an integer

  // Send the AT command using the stored key
  if (parent_->send_update_command(key_, int_value))
  {
    ESP_LOGI(TAG, "Sent setting number: %i", int_value);
  }
}

void AutoslideOnOffSwitch::write_state(bool value)
{
  int protocol_value = -1;

  // Determine the protocol value based on the key, as 'ON' (true) doesn't always mean '1'
  if (key_ == 'e')
  { // Open Speed: ON(true) = SLOW(1), OFF(false) = FAST(0)
    protocol_value = value ? OPEN_SPEED_SLOW : OPEN_SPEED_FAST;
  }
  else if (key_ == 'g')
  { // Secure Pet: ON(true) = OFF(1), OFF(false) = ON(0)
    protocol_value = value ? SECURE_PET_OFF : SECURE_PET_ON;
  }

  if (protocol_value != -1)
  {
    // Send the AT command using the stored key
    if (parent_->send_update_command(key_, protocol_value))
    {
      ESP_LOGI(TAG, "Sent on/off state: %s", value ? "on" : "off");
    }
  }
  else
  {
    ESP_LOGE(TAG, "Unknown switch key '%c' in write_state", key_);
  }
}

} // namespace autoslide_door
} // namespace esphome
