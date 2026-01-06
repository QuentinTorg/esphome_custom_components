#include "autoslide_door.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <sstream>
#include <cmath>  // for lroundf

namespace esphome {
namespace autoslide_door {

static const char *const TAG = "autoslide_door";
static const uint32_t COMMAND_TIMEOUT_MS = 10000; // 10 s timeout with no response
static const uint32_t POLL_INTERVAL_MS   = 10000; // 1 s periodic poll
static const uint32_t OFFLINE_TIMEOUT_MS = 60000; // 60 s without RX => offline

// --- Helper Functions for String Conversion (from .h) ---

std::string AutoslideDoor::mode_to_string(AutoslideMode mode) const
{
  switch (mode)
  {
    case AutoslideMode::AUTO:
      return "Auto";
    case AutoslideMode::STACK:
      return "Stack";
    case AutoslideMode::LOCK:
      return "Lock";
    case AutoslideMode::PET:
      return "Pet";
    default:
      return "Unknown";
  }
}

std::string AutoslideDoor::motion_state_to_string(AutoslideMotionState state) const
{
  switch (state)
  {
    case AutoslideMotionState::STOPPED:
      return "Stopped";
    case AutoslideMotionState::OPENING:
      return "Opening";
    case AutoslideMotionState::CLOSING:
      return "Closing";
    default:
      return "Unknown";
  }
}

bool AutoslideDoor::speed_to_bool(AutoslideOpenSpeed speed) const
{
  return speed == AutoslideOpenSpeed::SLOW;
}

bool AutoslideDoor::secure_pet_to_bool(AutoslideSecurePet pet) const
{
  return pet == AutoslideSecurePet::OFF;
}

// --- AutoslideDoor Component Implementation ---

void AutoslideDoor::setup()
{
  ESP_LOGCONFIG(TAG, "Setting up Autoslide Door Component...");
  state_ = {};
  last_rx_time_ms_ = 0;
  last_poll_time_ms_ = esphome::millis();

  receive_buffer_.reserve(256);

  // as soon as its available, request latest state from door
  queued_state_request_ = true;
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
    ESP_LOGCONFIG(TAG, "    open_speed %s", state_.open_speed == AutoslideOpenSpeed::FAST ? "fast" : "slow");
    ESP_LOGCONFIG(TAG, "    secure_pet: %s", state_.secure_pet == AutoslideSecurePet::ON ? "on" : "off");
    ESP_LOGCONFIG(TAG, "    open_hold_duration: %i", state_.open_hold_duration);
    ESP_LOGCONFIG(TAG, "    open_force: %i", state_.open_force);
    ESP_LOGCONFIG(TAG, "    close_force: %i", state_.close_force);
    ESP_LOGCONFIG(TAG, "    close_end_force: %i", state_.close_end_force);
    ESP_LOGCONFIG(TAG, "    motion_state: %s", motion_state_to_string(state_.motion_state).c_str());
    ESP_LOGCONFIG(TAG, "    lock_state: %s", state_.lock_state == AutoslideLockedState::LOCKED ? "locked" : "unlocked");
    ESP_LOGCONFIG(TAG, "    motion_trigger: %i", state_.motion_trigger);
}

void AutoslideDoor::loop()
{
  // 1. Read incoming UART data
  size_t bytes_read{0};
  size_t commands_received{0};
  uint8_t byte;
  while (available() and bytes_read < 256 and commands_received < 2)
  {
    if (!read_byte(&byte))
    {
      break;
    }
    ++bytes_read;

    // The Autoslide protocol uses the escape character (0x1B) to terminate commands.
    if (byte == 0x1B)
    {
      // Complete command received. Process it.
      if (!receive_buffer_.empty())
      {
        handle_incoming_command(receive_buffer_);
        ++commands_received;
      }
      // Clear buffer for the next command
      receive_buffer_.clear();
    }
    else if (byte == '\r' || byte == '\n')
    {
      // Ignore stray CR/LF if they ever appear
      ESP_LOGV(TAG, "Received unexpected whitespace character from serial bus");
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
      block_warned_ = false;
      queued_state_request_ = true;
    }
  }

  if (queued_upsend_reply_)
  {
    send_upsend_reply();
    App.feed_wdt();  // let scheduler breathe after TX
    return;          // do one heavy action per tick
  }

  // handle queued commands if we aren't waiting on anything anymore
  if (!awaiting_result_from_update_)
  {
    if (queued_trigger_)
    {
      trigger_open();
      App.feed_wdt();
      return;
    }
    else if (queued_mode_.has_value())
    {
      set_mode(queued_mode_.value());
      App.feed_wdt();
      return;
    }
    else if (queued_state_request_ or now - last_poll_time_ms_ >= POLL_INTERVAL_MS)
    {
      request_all_settings();
      App.feed_wdt();
      return;
    }
  }

  if (queued_publish_)
  {
    publish_current_state();
    App.feed_wdt();
    return;
  }

  // 4. Offline detection (no RX for a while)
  if (state_.connected && last_rx_time_ms_ != 0 && (now - last_rx_time_ms_ >= OFFLINE_TIMEOUT_MS))
  {
    ESP_LOGW(TAG, "No UART activity for %u ms, marking Autoslide disconnected.", OFFLINE_TIMEOUT_MS);
    state_.connected = false;
    if (connected_sensor_ != nullptr)
    {
      connected_sensor_->publish_state(false);
    }
  }
}

void AutoslideDoor::trigger_open(const bool defer)
{
  if (not defer and send_update_command('b', static_cast<int>(AutoslideTrigger::INDOOR)))
  {
    ESP_LOGD(TAG, "Sent Master Open Trigger (b:1)");
    queued_trigger_ = false;
  }
  else
  {
    ESP_LOGD(TAG, "Trigger open failed, queuing open trigger");
    queued_trigger_ = true;
  }
}

void AutoslideDoor::set_mode(const AutoslideMode& mode, const bool defer)
{
  if (not defer and send_update_command('a', static_cast<int>(mode)))
  {
    ESP_LOGD(TAG, "Sent mode %s (%i)", mode_to_string(mode).c_str(), static_cast<int>(mode));
    queued_mode_ = {};
  }
  else
  {
    ESP_LOGD(TAG, "Changing mode failed, queuing mode change to %s", mode_to_string(mode).c_str());
    queued_mode_ = mode;
  }
}

// --- Utility Functions ---

bool AutoslideDoor::send_update_command(char key, int value)
{
  if (awaiting_result_from_update_)
  {
    if (not block_warned_) {
      ESP_LOGW(TAG, "Cannot send command ('%c':%d). Waiting for AT+RESULT from previous command.", key, value);
      block_warned_ = true;  // only warn once per busy window
    }
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

  std::string printable = command;
  for (auto &ch : printable)
  {
      if (ch == 0x1B)
      {
          ch = '\\';
      }
  }
  ESP_LOGD(TAG, "Sending command: %s", printable.c_str());

  write_str(command.c_str());
  App.feed_wdt();

  const auto now_ms = esphome::millis();
  awaiting_result_from_update_ = true;
  last_command_sent_time_ms_ = now_ms;

  inflight_key_ = key;
  inflight_value_ = value;

  return true;
}

void AutoslideDoor::send_upsend_reply()
{
  const std::string command = "AT+REPLY,r:1" + std::string(1, 0x1B);
  std::string printable = command;
  for (auto &ch : printable)
  {
      if (ch == 0x1B)
      {
          ch = '\\';
      }
  }
  ESP_LOGD(TAG, "Sending UPSEND Reply: %s", printable.c_str());
  write_str(command.c_str());
  App.feed_wdt();
  queued_upsend_reply_ = false;
}

void AutoslideDoor::request_all_settings()
{
  if (send_update_command('d', 0))
  {
    ESP_LOGI(TAG, "Requesting all door settings (d:0)...");
    queued_state_request_ = false;
  }
  last_poll_time_ms_ = esphome::millis();
}

void AutoslideDoor::handle_incoming_command(const std::string &command)
{
  ESP_LOGV(TAG, "Received raw command: %s", command.c_str());

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
    // return early so we don't mark connection
    return;
  }

  // refresh the connection status
  last_rx_time_ms_ = esphome::millis();
  if (!state_.connected)
  {
    state_.connected = true;
    if (connected_sensor_ != nullptr)
    {
      connected_sensor_->publish_state(true);
    }
  }
}

void AutoslideDoor::handle_result_command(const std::string &payload)
{
  ESP_LOGD(TAG, "Received AT+RESULT with payload: %s", payload.c_str());

  awaiting_result_from_update_ = false;
  block_warned_ = false;

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

    if (colon_pos != 1)
    {
        ESP_LOGW(TAG, "Malformed key-value pair, key had length of more than one: %s", key_value_pair.c_str());
        continue;
    }

    const char key = key_value_pair[0];
    const int value = atoi(key_value_pair.c_str() + colon_pos + 1);

    if (key == 'r')
    {
      if (value == 1)
      {
        ESP_LOGI(TAG, "Command acknowledged successfully.");
        if (inflight_key_ != 0 and inflight_key_ != 'b' and inflight_key_ != 'd')
        {
          // Only for writable keys that represent settings (skip 'b' and 'd')
          update_state(inflight_key_, inflight_value_);
        }
      }
      else
      {
        ESP_LOGE(TAG, "Command failed to execute (r:%d).", value);
        queued_state_request_ = true; // request new state if response fails
      }
    }
    else
    {
      update_state(key, value);
    }
  }

  inflight_key_ = 0;
  queued_publish_ = true;
}

void AutoslideDoor::handle_upsend_command(const std::string &payload)
{
  ESP_LOGD(TAG, "Received AT+UPSEND (Status Update) with payload: %s", payload.c_str());

  queued_upsend_reply_ = true;

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
  queued_publish_ = true;
}

void AutoslideDoor::update_state(const char key, const int value)
{
  switch (key)
  {
    case 'a': state_.door_mode = static_cast<AutoslideMode>(value); break;
    case 'e': state_.open_speed = static_cast<AutoslideOpenSpeed>(value); break;
    case 'g': state_.secure_pet = static_cast<AutoslideSecurePet>(value); break;
    case 'j': state_.open_hold_duration = static_cast<uint8_t>(value); break;
    case 'C': state_.open_force = static_cast<uint8_t>(value); break;
    case 'z': state_.close_force = static_cast<uint8_t>(value); break;
    case 'A': state_.close_end_force = static_cast<uint8_t>(value); break;
    case 'm': state_.motion_state = static_cast<AutoslideMotionState>(value); break;
    case 'c': state_.lock_state = static_cast<AutoslideLockedState>(value); break;
    case 'n': state_.motion_trigger = static_cast<uint8_t>(value); break;
    default:
      if (key != 'r')
      {
        ESP_LOGW(TAG, "Received unknown key '%c' with value %d", key, value);
      }
      break;
  }
}

void AutoslideDoor::publish_current_state(bool force)
{
  ESP_LOGV(TAG, "Publishing current state to ESPHome entities (force=%s)...", force ? "true" : "false");

  if (not have_published_once_)
  {
      // override force if we haven't published anything yet
      ESP_LOGV(TAG, "Overriding publish force to true because initial states have not yet been published");
      force = true;
  }

  // Mode select
  if (mode_select_ != nullptr) {
    if (force || state_.door_mode != last_published_state_.door_mode) {
      mode_select_->publish_state(mode_to_string(state_.door_mode));
    }
  }

  // Open speed
  if (open_speed_switch_ != nullptr) {
    if (force || state_.open_speed != last_published_state_.open_speed) {
      open_speed_switch_->publish_state(speed_to_bool(state_.open_speed));
    }
  }

  // Secure pet
  if (secure_pet_switch_ != nullptr) {
    if (force || state_.secure_pet != last_published_state_.secure_pet) {
      secure_pet_switch_->publish_state(secure_pet_to_bool(state_.secure_pet));
    }
  }

  // Open hold duration
  if (open_hold_number_ != nullptr) {
    if (force || state_.open_hold_duration != last_published_state_.open_hold_duration) {
      open_hold_number_->publish_state(state_.open_hold_duration);
    }
  }

  // Forces
  if (open_force_number_ != nullptr) {
    if (force || state_.open_force != last_published_state_.open_force) {
      open_force_number_->publish_state(state_.open_force);
    }
  }
  if (close_force_number_ != nullptr) {
    if (force || state_.close_force != last_published_state_.close_force) {
      close_force_number_->publish_state(state_.close_force);
    }
  }
  if (close_end_force_number_ != nullptr) {
    if (force || state_.close_end_force != last_published_state_.close_end_force) {
      close_end_force_number_->publish_state(state_.close_end_force);
    }
  }

  // Motion state text
  if (motion_state_sensor_ != nullptr) {
    if (force || state_.motion_state != last_published_state_.motion_state) {
      const char *motion_str = "Unknown";
      switch (state_.motion_state) {
        case AutoslideMotionState::STOPPED: motion_str = "Stopped"; break;
        case AutoslideMotionState::OPENING: motion_str = "Opening"; break;
        case AutoslideMotionState::CLOSING: motion_str = "Closing"; break;
        default: break;
      }
      motion_state_sensor_->publish_state(motion_str);
    }
  }

  // Lock state text
  if (lock_state_sensor_ != nullptr) {
    if (force || state_.lock_state != last_published_state_.lock_state) {
      const char *lock_str = (state_.lock_state == AutoslideLockedState::LOCKED) ? "Locked" : "Unlocked";
      lock_state_sensor_->publish_state(lock_str);
    }
  }

  // Connection sensor
  if (connected_sensor_ != nullptr) {
    if (force || state_.connected != last_published_state_.connected) {
      connected_sensor_->publish_state(state_.connected);
    }
  }

  // Update cache and mark first publish done
  last_published_state_ = state_;
  have_published_once_ = true;
  queued_publish_ = false;
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
void AutoslideDoor::set_connected_sensor(binary_sensor::BinarySensor *sensor) { connected_sensor_ = sensor; }

// --- Custom Entity Control Implementations ---

void AutoslideModeSelect::control(const std::string &value)
{
  AutoslideMode mode_value = AutoslideMode::UNKNOWN;
  if (value == "Auto")
  {
      mode_value = AutoslideMode::AUTO;
  }
  else if (value == "Stack")
  {
      mode_value = AutoslideMode::STACK;
  }
  else if (value == "Lock")
  {
      mode_value = AutoslideMode::LOCK;
  }
  else if (value == "Pet")
  {
      mode_value = AutoslideMode::PET;
  }

  if (mode_value != AutoslideMode::UNKNOWN && parent_ != nullptr)
  {
      ESP_LOGI(TAG, "Sending mode command: %s (converted to %s)",
               value.c_str(),
               parent_->mode_to_string(mode_value).c_str());

      parent_->set_mode(mode_value, true);
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
    protocol_value = static_cast<int>(value ? AutoslideOpenSpeed::SLOW : AutoslideOpenSpeed::FAST);
  }
  else if (key_ == 'g')
  { // Secure Pet: ON(true) = OFF(1), OFF(false) = ON(0)
    protocol_value = static_cast<int>(value ? AutoslideSecurePet::OFF : AutoslideSecurePet::ON);
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
    parent_->trigger_open(true);
  else
    ESP_LOGE(TAG, "Open button pressed but parent not set");
}

} // namespace autoslide_door
} // namespace esphome
