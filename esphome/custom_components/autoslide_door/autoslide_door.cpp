#include "autoslide_door.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <sstream>
#include <cmath>  // for lroundf

namespace esphome {
namespace autoslide_door {

static const char *const TAG = "autoslide_door";
static const uint32_t COMMAND_TIMEOUT_MS = 15000; // 15 s as per the guide
static const uint32_t POLL_INTERVAL_MS   = 30000; // 30 s periodic poll
static const uint32_t OFFLINE_TIMEOUT_MS = 60000; // 60 s without RX => offline

// --- Helper Functions for String Conversion (from .h) ---

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
  return speed == OPEN_SPEED_SLOW;
}

bool AutoslideDoor::secure_pet_to_bool(AutoslideSecurePet pet) const
{
  return pet == SECURE_PET_OFF;
}

// --- AutoslideDoor Component Implementation ---

void AutoslideDoor::setup()
{
  ESP_LOGCONFIG(TAG, "Setting up Autoslide Door Component...");
  state_ = {};
  connected_ = false;
  last_rx_time_ms_ = 0;
  last_poll_time_ms_ = esphome::millis();

  request_all_settings();
}

float AutoslideDoor::get_setup_priority() const
{
  return setup_priority::BUS;
}

void AutoslideDoor::dump_config()
{
    ESP_LOGCONFIG(TAG, "autoslide component");
    ESP_LOGCONFIG(TAG, "  awaiting_result_from_update: %s", awaiting_result_from_update_ ? "true" : "false");
    ESP_LOGCONFIG(TAG, "  last_command_sent_time_ms: %u", last_command_sent_time_ms_);
    ESP_LOGCONFIG(TAG, "  now:                       %u", esphome::millis());
    ESP_LOGCONFIG(TAG, "  state:");
    ESP_LOGCONFIG(TAG, "    mode: %s", mode_to_string(state_.door_mode).c_str());
    ESP_LOGCONFIG(TAG, "    open_speed %s", state_.open_speed == OPEN_SPEED_FAST ? "fast" : "slow");
    ESP_LOGCONFIG(TAG, "    secure_pet: %s", state_.secure_pet == SECURE_PET_ON ? "on" : "off");
    ESP_LOGCONFIG(TAG, "    open_hold_duration: %i", state_.open_hold_duration);
    ESP_LOGCONFIG(TAG, "    open_force: %i", state_.open_force);
    ESP_LOGCONFIG(TAG, "    close_force: %i", state_.close_force);
    ESP_LOGCONFIG(TAG, "    close_end_force: %i", state_.close_end_force);
    ESP_LOGCONFIG(TAG, "    motion_state: %s", motion_state_to_string(state_.motion_state).c_str());
    ESP_LOGCONFIG(TAG, "    lock_state: %s", state_.lock_state == STATE_LOCKED ? "locked" : "unlocked");
    ESP_LOGCONFIG(TAG, "    motion_trigger: %i", state_.motion_trigger);
}

void AutoslideDoor::loop()
{
  // 1. Read incoming UART data
  while (available())
  {
    uint8_t byte;
    if (!read_byte(&byte))  // defensive: skip if nothing read
      break;

    // The Autoslide protocol uses the escape character (0x1B) to terminate commands.
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
    else if (byte == '\r' || byte == '\n')
    {
      // Ignore stray CR/LF if they ever appear
      ESP_LOGI(TAG, "Received unexpected whitespace character from serial bus");
    }
    else
    {
      // Append byte to the receive buffer
      receive_buffer_ += static_cast<char>(byte);

      // guard against buffer overrun
      if (receive_buffer_.size() > 256)
      {
        ESP_LOGW(TAG, "RX buffer overflow, dropping partial frame");
        receive_buffer_.clear();
      }
    }
  }

  const auto now = esphome::millis();

  // 2. Handle Command Timeout
  if (awaiting_result_from_update_)
  {
    if (now - last_command_sent_time_ms_ > COMMAND_TIMEOUT_MS)
    {
      ESP_LOGE(TAG, "Command timeout! Did not receive AT+RESULT within %u ms.", COMMAND_TIMEOUT_MS);
      awaiting_result_from_update_ = false;
    }
  }

  // 3. Periodic poll to keep link/state fresh
  if (!awaiting_result_from_update_ && (now - last_poll_time_ms_ >= POLL_INTERVAL_MS)) {
    request_all_settings();
    last_poll_time_ms_ = now;
  }

  // 4. Offline detection (no RX for a while)
  if (connected_ && last_rx_time_ms_ != 0 && (now - last_rx_time_ms_ >= OFFLINE_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "No UART activity for %u ms, marking Autoslide disconnected.", OFFLINE_TIMEOUT_MS);
    connected_ = false;
    if (connected_sensor_ != nullptr)
      connected_sensor_->publish_state(false);
  }
}

void AutoslideDoor::trigger_open()
{
  if (send_update_command('b', TRIGGER_MASTER))
  {
    ESP_LOGD(TAG, "Sent Master Open Trigger (b:0)");
  }
}

// --- Utility Functions ---

bool AutoslideDoor::send_update_command(char key, int value)
{
  if (awaiting_result_from_update_)
  {
    ESP_LOGW(TAG, "Cannot send command ('%c':%d). Waiting for AT+RESULT from previous command.", key, value);
    return false;
  }

  std::string command = "AT+UPDATE,";
  command += key;
  command += ":";
  if (key == 'j' && value >= 0 && value <= 9)
  {
      // for j value only output must be 2 digits
      command += '0';
  }
  command += std::to_string(value);
  command += (char) 0x1B; // Escape character

  ESP_LOGD(TAG, "Sending command: %s", command.c_str());

  write_str(command.c_str());

  const auto now_ms = esphome::millis();
  awaiting_result_from_update_ = true;
  last_command_sent_time_ms_ = now_ms;
  last_poll_time_ms_ = now_ms;
  return true;
}

void AutoslideDoor::send_upsend_reply()
{
  const std::string command = "AT+REPLY,r:1" + std::string(1, 0x1B);
  ESP_LOGD(TAG, "Sending UPSEND Reply: %s", command.c_str());
  write_str(command.c_str());
}

void AutoslideDoor::request_all_settings()
{
  if (send_update_command('d', 0))
  {
    ESP_LOGI(TAG, "Requesting initial door settings (d:0)...");
  }
}

void AutoslideDoor::handle_incoming_command(const std::string &command)
{
  ESP_LOGV(TAG, "Received raw command: %s", command.c_str());

  // refresh the connection status
  last_rx_time_ms_ = esphome::millis();
  if (!connected_)
  {
    connected_ = true;
    if (connected_sensor_ != nullptr)
    {
      connected_sensor_->publish_state(true);
    }
  }

  if (command.length() < 3 || command.substr(0, 3) != "AT+")
  {
    ESP_LOGE(TAG, "Invalid AT command prefix: %s", command.c_str());
    return;
  }

  size_t comma_pos = command.find(',');
  std::string command_type;
  std::string payload;

  if (comma_pos == std::string::npos)
  {
    command_type = command.substr(3);
  }
  else
  {
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
    ESP_LOGV(TAG, "Received AT+REPLY from Autoslide. Acknowledged.");
  }
  else
  {
    ESP_LOGW(TAG, "Unknown AT command type received: %s", command_type.c_str());
  }
}

void AutoslideDoor::handle_result_command(const std::string &payload)
{
  ESP_LOGD(TAG, "Received AT+RESULT with payload: %s", payload.c_str());

  awaiting_result_from_update_ = false;

  std::stringstream ss(payload);
  std::string key_value_pair;

  while (std::getline(ss, key_value_pair, ','))
  {
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
      update_state(key, value);
    }
  }
  publish_current_state();
}

void AutoslideDoor::handle_upsend_command(const std::string &payload)
{
  ESP_LOGD(TAG, "Received AT+UPSEND (Status Update) with payload: %s", payload.c_str());

  send_upsend_reply();

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

    update_state(key, value);
  }
  publish_current_state();
}

void AutoslideDoor::update_state(char key, int value)
{
  switch (key)
  {
    case 'a': state_.door_mode = (AutoslideMode) value; break;
    case 'e': state_.open_speed = (AutoslideOpenSpeed) value; break;
    case 'g': state_.secure_pet = (AutoslideSecurePet) value; break;
    case 'j': state_.open_hold_duration = (uint8_t) value; break;
    case 'C': state_.open_force = (uint8_t) value; break;
    case 'z': state_.close_force = (uint8_t) value; break;
    case 'A': state_.close_end_force = (uint8_t) value; break;

    case 'm': state_.motion_state = (AutoslideMotionState) value; break;
    case 'c': state_.lock_state = (AutoslideLockedState) value; break;
    case 'n': state_.motion_trigger = (uint8_t) value; break;
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

  if (mode_select_ != nullptr)
  {
    mode_select_->publish_state(mode_to_string(state_.door_mode));
  }
  if (open_speed_switch_ != nullptr)
  {
    open_speed_switch_->publish_state(speed_to_bool(state_.open_speed));
  }
  if (secure_pet_switch_ != nullptr)
  {
    secure_pet_switch_->publish_state(secure_pet_to_bool(state_.secure_pet));
  }
  if (open_hold_number_ != nullptr)
  {
    open_hold_number_->publish_state(state_.open_hold_duration);
  }
  if (open_force_number_ != nullptr)
  {
    open_force_number_->publish_state(state_.open_force);
  }
  if (close_force_number_ != nullptr)
  {
    close_force_number_->publish_state(state_.close_force);
  }
  if (close_end_force_number_ != nullptr)
  {
    close_end_force_number_->publish_state(state_.close_end_force);
  }
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
  if (lock_state_sensor_ != nullptr)
  {
    const char *lock_str = (state_.lock_state == STATE_LOCKED) ? "Locked" : "Unlocked";
    lock_state_sensor_->publish_state(lock_str);
  }
  if (busy_sensor_ != nullptr)
  {
    busy_sensor_->publish_state(awaiting_result_from_update_);
  }
  if (connected_sensor_ != nullptr)
  {
    connected_sensor_->publish_state(connected_);
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
void AutoslideDoor::set_open_button(button::Button *button) { open_button_ = button; }
void AutoslideDoor::set_busy_sensor(binary_sensor::BinarySensor *sensor) { busy_sensor_ = sensor; }
void AutoslideDoor::set_connected_sensor(binary_sensor::BinarySensor *sensor) { connected_sensor_ = sensor; }

// --- Custom Entity Control Implementations ---

void AutoslideModeSelect::control(const std::string &value)
{
  int mode_value = -1;
  if (value == "Auto")      mode_value = MODE_AUTO;
  else if (value == "Stack") mode_value = MODE_STACK;
  else if (value == "Lock")  mode_value = MODE_LOCK;
  else if (value == "Pet")   mode_value = MODE_PET;

  if (mode_value != -1 && parent_ != nullptr)
  {
    if (parent_->send_update_command('a', mode_value))
    {
      ESP_LOGI(TAG, "Sent mode command: %s (converted to %s)",
               value.c_str(),
               parent_->mode_to_string((AutoslideMode) mode_value).c_str());
    }
  }
  else
  {
    ESP_LOGE(TAG, "Invalid door mode selected or parent not set: %s", value.c_str());
  }
}

void AutoslideSettingNumber::control(float value)
{
  if (parent_ == nullptr || key_ == 0)
  {
    ESP_LOGE(TAG, "Number control missing parent or key");
    return;
  }

  int int_value = (int) lroundf(value);

  if (parent_->send_update_command(key_, int_value))
  {
    ESP_LOGI(TAG, "Sent setting number (%c): %i", key_, int_value);
  }
}

void AutoslideOnOffSwitch::write_state(bool value)
{
  if (parent_ == nullptr || key_ == 0)
  {
    ESP_LOGE(TAG, "Switch write_state missing parent or key");
    return;
  }

  int protocol_value = -1;

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
    if (parent_->send_update_command(key_, protocol_value))
    {
      ESP_LOGI(TAG, "Sent on/off state (%c): %s", key_, value ? "on" : "off");
    }
  }
  else
  {
    ESP_LOGE(TAG, "Unknown switch key '%c' in write_state", key_);
  }
}

void AutoslideOpenButton::press_action()
{
  if (parent_ != nullptr)
    parent_->trigger_open();
  else
    ESP_LOGE(TAG, "Open button pressed but parent not set");
}

} // namespace autoslide_door
} // namespace esphome
