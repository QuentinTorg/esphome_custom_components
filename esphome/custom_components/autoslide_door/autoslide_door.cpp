#include "autoslide_door.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cstdio>   // snprintf
#include <cstdlib>  // strtol
#include <cstring>  // strchr

namespace esphome {
namespace autoslide_door {

static const uint32_t COMMAND_TIMEOUT_MS = 1000;   // timeout with no response
static const uint32_t OFFLINE_TIMEOUT_MS = 5000; // 2 missed polls indicate timeout
static const uint32_t STARTUP_DELAY_MS = 25000; // delay on startup waiting for first command

// --- Helper Functions for String Conversion (from .h) ---

std::string mode_to_string(AutoslideMode mode)
{
  switch (mode)
  {
    case AutoslideMode::AUTO:  return "Auto";
    case AutoslideMode::STACK: return "Stack";
    case AutoslideMode::LOCK:  return "Lock";
    case AutoslideMode::PET:   return "Pet";
    default:                   return "Unknown";
  }
}

std::string motion_state_to_string(AutoslideMotionState state)
{
  switch (state)
  {
    case AutoslideMotionState::STOPPED: return "Stopped";
    case AutoslideMotionState::OPENING: return "Opening";
    case AutoslideMotionState::CLOSING: return "Closing";
    default:                            return "Unknown";
  }
}

bool speed_to_bool(const AutoslideOpenSpeed speed)
{
  return speed == AutoslideOpenSpeed::SLOW;
}

AutoslideOpenSpeed bool_to_speed(const bool speed_bool)
{
  return {speed_bool ? AutoslideOpenSpeed::SLOW : AutoslideOpenSpeed::FAST};
}

bool secure_pet_to_bool(const AutoslideSecurePet pet)
{
  return pet == AutoslideSecurePet::OFF;
}

AutoslideSecurePet bool_to_secure_pet(const bool pet_bool)
{
  return {pet_bool ? AutoslideSecurePet::OFF : AutoslideSecurePet::ON};
}

// --- AutoslideDoor Component Implementation ---

float AutoslideDoor::get_setup_priority() const
{
  return setup_priority::BUS;
}

void AutoslideDoor::dump_config()
{
  ESP_LOGCONFIG(TAG, "autoslide component");
  ESP_LOGCONFIG(TAG, "  awaiting_result_from_update: %s", inflight_update_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  last_command_sent_time_ms: %u", inflight_update_ ? inflight_update_->sent_time_ms : 0);
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

void AutoslideDoor::setup()
{
  ESP_LOGCONFIG(TAG, "Setting up Autoslide Door Component...");

  receive_buffer_.reserve(256);

  // don't send commands until first command arrives or startup delay completes
  startup_timeout_stamp_ms = millis() + STARTUP_DELAY_MS;
}

void AutoslideDoor::loop()
{
  // 1) Read incoming UART data (bounded per tick)
  size_t bytes_read{0};
  size_t commands_received{0};
  uint8_t byte;
  while (available() && bytes_read < 128 && commands_received < 2)
  {
    if (!read_byte(&byte))
    {
        break;
    }
    ++bytes_read;

    if (byte == 0x1B) // Escape terminator
    {
      // Temporary debug sensor
      if (debug_uart_sensor_ != nullptr)
      {
        debug_uart_sensor_->publish_state(receive_buffer_);
      }

      if (!receive_buffer_.empty())
      {
        handle_incoming_command(receive_buffer_);
        ++commands_received;
      }
      receive_buffer_.clear();
    }
    else if (byte == '\r' || byte == '\n')
    {
      ESP_LOGV(TAG, "Received unexpected whitespace character from serial bus");
    }
    else
    {
      receive_buffer_ += static_cast<char>(byte);
      if (receive_buffer_.size() > 128)
      {
        ESP_LOGW(TAG, "RX buffer overflow, dropping partial frame");
        receive_buffer_.clear();
      }
    }
  }

  const auto now_ms = esphome::millis();

  // wait for door to send first command before sending any updates.
  // If we reach timeout, assume the door is already running
  if (now_ms < startup_timeout_stamp_ms)
  {
      return;
  }

  // 2) Handle Command Timeout
  if (inflight_update_ and now_ms - inflight_update_->sent_time_ms > COMMAND_TIMEOUT_MS)
  {
    ESP_LOGE(TAG, "Command timeout! Did not receive AT+RESULT within %u ms.", COMMAND_TIMEOUT_MS);
    inflight_update_.reset();
    if (state_.connected)
    {
      ESP_LOGW(TAG, "No UART response within %u ms, marking Autoslide disconnected.", COMMAND_TIMEOUT_MS);
      state_.connected = false;
    }

    // Schedule a delayed REQUEST_ALL with exponential backoff
    const uint8_t capped_attempts = retry_attempts_ > 5 ? 5 : retry_attempts_;
    uint32_t delay_ms = 1000u << capped_attempts;
    if (delay_ms > 30000u)
    {
      delay_ms = 30000u;
    }
    retry_backoff_ = RetryBackoff{.start_ms = now_ms, .delay_ms = delay_ms};
    retry_attempts_++;
  }

  // 3) Reply to UPSEND (ack) if needed
  if (queued_upsend_reply_)
  {
    send_upsend_reply();
  }

  // 3b) If backoff elapsed, queue a single REQUEST_ALL retry
  if (retry_backoff_.has_value() &&
      (uint32_t) (now_ms - retry_backoff_->start_ms) >= retry_backoff_->delay_ms)
  {
    ESP_LOGI(TAG, "Retry backoff elapsed. Queueing full status request.");
    request_set_key_value(AutoslideKey::REQUEST_ALL, 0);
    retry_backoff_.reset();
  }

  // 4) Reconcile desired state -> send exactly one command if idle
  if (send_next_update())
  {
    ESP_LOGV(TAG, "Sent next update");
  }

  // 5) Offline detection (only when we're waiting on a response)
  if (state_.connected && inflight_update_)
  {
    const uint32_t sent_ms = inflight_update_->sent_time_ms;
    const bool rx_after_send = last_rx_time_ms_.has_value() && last_rx_time_ms_.value() >= sent_ms;
    if (!rx_after_send && (now_ms - sent_ms > OFFLINE_TIMEOUT_MS))
    {
      ESP_LOGW(TAG, "No UART response for %u ms after command, marking Autoslide disconnected.", OFFLINE_TIMEOUT_MS);
      if (!last_rx_time_ms_.has_value())
      {
        ESP_LOGE(TAG, "last_rx_time_ms_: has no value");
      }
      else
      {
        ESP_LOGW(TAG, "last_rx_time_ms_:%u, last_tx_time_ms_:%u", last_rx_time_ms_.value(), sent_ms);
      }
      state_.connected = false;
    }
  }

  // 6) Request setting update
  if (not first_poll_complete_)
  {
    ESP_LOGI(TAG, "Poll interval expired. Queueing full status request."); // <--- ADD THIS
    request_set_key_value(AutoslideKey::REQUEST_ALL, 0);
    first_poll_complete_ = true;
  }

  // 7) Publish current state if we make it this far
  publish_current_state();
}

bool AutoslideDoor::publish_current_state(const bool full_publish)
{
    static uint32_t last_publish_ms = 0;
    if (esphome::millis() - last_publish_ms > 5000)
    {
        ESP_LOGV(TAG, "Publishing current state to ESPHome entities (full_publish=%s)...", full_publish ? "true" : "false");
        last_publish_ms = esphome::millis();
    }

  // only publish one element per fuction call unless force publish

  // Mode select
  if (mode_select_ != nullptr)
  {
    const std::string current_mode_str = mode_to_string(state_.door_mode);
    // Check if the Entity (ESPHome) thinks differently than the Door (Hardware)
    if (full_publish || mode_select_->current_option() != current_mode_str)
    {
      ESP_LOGV(TAG, "!!!!!!!!!!!!!!!!!!publishing current mode: %s, select mode: %s", current_mode_str.c_str(), mode_select_->current_option());
      mode_select_->publish_state(current_mode_str);
      // If we found a mismatch and fixed it, return true to yield (throttle updates)
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Open speed
  if (open_speed_switch_ != nullptr)
  {
    const bool current_speed_bool = speed_to_bool(state_.open_speed);
    if (full_publish || open_speed_switch_->state != current_speed_bool)
    {
      open_speed_switch_->publish_state(current_speed_bool);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Secure pet
  if (secure_pet_switch_ != nullptr)
  {
    const bool current_pet_bool = secure_pet_to_bool(state_.secure_pet);
    if (full_publish || secure_pet_switch_->state != current_pet_bool)
    {
      secure_pet_switch_->publish_state(current_pet_bool);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Open hold duration
  if (open_hold_number_ != nullptr)
  {
    // Cast to int to ensure we aren't comparing 5.0 to 5 and getting false negatives
    if (full_publish || (int)open_hold_number_->state != state_.open_hold_duration)
    {
      open_hold_number_->publish_state(state_.open_hold_duration);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Forces
  if (open_force_number_ != nullptr)
  {
    if (full_publish || (int)open_force_number_->state != state_.open_force)
    {
      open_force_number_->publish_state(state_.open_force);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  if (close_force_number_ != nullptr)
  {
    if (full_publish || (int)close_force_number_->state != state_.close_force)
    {
      close_force_number_->publish_state(state_.close_force);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  if (close_end_force_number_ != nullptr)
  {
    if (full_publish || (int)close_end_force_number_->state != state_.close_end_force)
    {
      close_end_force_number_->publish_state(state_.close_end_force);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Motion state text
  if (motion_state_sensor_ != nullptr)
  {
    const std::string current_motion = motion_state_to_string(state_.motion_state);
    if (full_publish || motion_state_sensor_->state != current_motion)
    {
      motion_state_sensor_->publish_state(current_motion);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Lock state text
  if (lock_state_sensor_ != nullptr)
  {
    const std::string current_lock = (state_.lock_state == AutoslideLockedState::LOCKED) ? "Locked" : "Unlocked";
    if (full_publish || lock_state_sensor_->state != current_lock)
    {
      lock_state_sensor_->publish_state(current_lock);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  // Connection sensor
  if (connected_sensor_ != nullptr)
  {
    if (full_publish || connected_sensor_->state != state_.connected)
    {
      connected_sensor_->publish_state(state_.connected);
      if (!full_publish)
      {
          return true;
      }
    }
  }

  return true;
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

// --- UART TX helpers ---

bool AutoslideDoor::send_update_command(const AutoslideKey key, const int value)
{
  if (inflight_update_)
  {
    ESP_LOGW(TAG, "Cannot send command ('%c':%d). Waiting for AT+RESULT from previous command.", key, value);
    return false;
  }

  char cmd[32];
  int n;
  if (key == AutoslideKey::HOLD_TIME)
  {
    n = snprintf(cmd, sizeof(cmd), "AT+UPDATE,%c:%02d%c", key, value, (char)0x1B);
  }
  else
  {
    n = snprintf(cmd, sizeof(cmd), "AT+UPDATE,%c:%d%c", key, value, (char)0x1B);
  }

  if (n <= 0 || n >= (int)sizeof(cmd))
  {
    ESP_LOGE(TAG, "Command formatting overflow for key '%c'", key);
    return false;
  }

  ESP_LOGV(TAG, "Sending update command: %s", cmd);

  write_str(cmd);

  inflight_update_ = {.key = key,
                      .value = value,
                      .sent_time_ms = esphome::millis()};

  return true;
}

void AutoslideDoor::send_upsend_reply()
{
  char cmd[20];
  int n = snprintf(cmd, sizeof(cmd), "AT+REPLY,r:1%c", (char)0x1B);

  if (n > 0 && n < (int)sizeof(cmd))
  {
    ESP_LOGD(TAG, "Sending UPSEND Reply");
    write_str(cmd);
  }
  queued_upsend_reply_ = false;
}

bool AutoslideDoor::send_next_update()
{
    if (inflight_update_ or queued_upsend_reply_)
    {
        return false;
    }

    for (auto& [key, value_maybe] : open_requests_)
    {
        if (value_maybe)
        {
            if (send_update_command(key, value_maybe.value()))
            {
                value_maybe = {};
                return true;
            }
            return false;
        }
    }

    return false;
}

// --- RX handling ---

void AutoslideDoor::handle_incoming_command(const std::string &command)
{
  ESP_LOGV(TAG, "Received raw command: %s", command.c_str());

  if (command.size() < 3 || command.compare(0, 3, "AT+") != 0)
  {
    ESP_LOGE(TAG, "Invalid AT command prefix: %s", command.c_str());
    return;
  }

  // split off the command type from the payload
  size_t comma_pos = command.find(',');
  std::string_view command_type;
  const char *payload_ptr = nullptr;
  size_t payload_len = 0;

  if (comma_pos == std::string::npos)
  {
    command_type = std::string_view(command).substr(3);
  }
  else
  {
    command_type = std::string_view(command).substr(3, comma_pos - 3);
    payload_ptr = command.c_str() + comma_pos + 1;
    payload_len = command.size() - (comma_pos + 1);
  }

  if (payload_ptr != nullptr)
  {
    ESP_LOGD(TAG, "Received AT+%.*s, payload: %.*s",
             static_cast<int>(command_type.size()), command_type.data(),
             static_cast<int>(payload_len), payload_ptr);
  }
  else
  {
    ESP_LOGD(TAG, "Received AT+%.*s (no payload)",
             static_cast<int>(command_type.size()), command_type.data());
  }

  // pass payload to the handler
  if (command_type == "RESULT")
  {
    parse_kv_payload(payload_ptr, payload_len);
    inflight_update_.reset();
  }
  else if (command_type == "UPSEND")
  {
    parse_kv_payload(payload_ptr, payload_len);
    queued_upsend_reply_ = true;
    startup_timeout_stamp_ms = 0; // first upsend means door is ready to receive commands
  }
  else if (command_type == "REPLY")
  {
    ESP_LOGV(TAG, "Received AT+REPLY from Autoslide. Acknowledged.");
  }
  else
  {
    ESP_LOGW(TAG, "Unknown AT command type received: %.*s", static_cast<int>(command_type.size()), command_type.data());
    return;
  }

  // refresh connection status
  last_rx_time_ms_ = esphome::millis();
  if (not state_.connected)
  {
    state_.connected = true;
  }
  retry_attempts_ = 0;
  retry_backoff_.reset();
}

// Split payload by commas, each token like "<k>:<v>" where k is one char
void AutoslideDoor::parse_kv_payload(const char *payload, const size_t len)
{
  if (payload == nullptr or len == 0)
  {
      return;
  }

  size_t i = 0;
  while (i < len)
  {
    // protect from any whitespace in the message
    while (i < len && (payload[i] == ' ' || payload[i] == '\t' || payload[i] == '\r' || payload[i] == '\n'))
    {
      i++;
    }
    if (i >= len)
    {
      break;
    }

    // token start
    char key = payload[i];
    // find colon after single key
    if (i + 1 >= len || payload[i + 1] != ':')
    {
      // colon does not exist where expected, skip to next comma
      while (i < len && payload[i] != ',')
      {
          ++i;
      }
      if (i < len && payload[i] == ',')
      {
          ++i;
      }
      ESP_LOGW(TAG, "Malformed key-value in payload (missing ':' at index %u, char '%c')", i, key);
      continue;
    }

    size_t val_start = i + 2;
    size_t j = val_start;
    while (j < len && payload[j] != ',')
    {
      ++j;
    }
    // parse integer value from payload[val_start..j)
    int sign = 1;
    int value = 0;
    size_t k = val_start;
    if (k < j && payload[k] == '-')
    {
        sign = -1; ++k;
    }
    for (; k < j; ++k)
    {
      if (payload[k] < '0' || payload[k] > '9')
      {
          break;
      }
      value = value * 10 + (payload[k] - '0');
    }
    value *= sign;

    // ESP_LOGV(TAG, "Received key, value: %c:%d", key, value);

    update_state(static_cast<AutoslideKey>(key), value);

    // move past comma
    i = (j < len) ? j + 1 : j;
  }
}

// --- State update and dirty-bit reconciliation ---

void AutoslideDoor::update_state(const AutoslideKey key, const int value)
{
  switch (key)
  {
    case AutoslideKey::MODE: state_.door_mode = static_cast<AutoslideMode>(value); break;
    case AutoslideKey::LOCK_STATE: state_.lock_state = static_cast<AutoslideLockedState>(value); break;
    case AutoslideKey::OPEN_SPEED: state_.open_speed = static_cast<AutoslideOpenSpeed>(value); break;
    case AutoslideKey::SECURE_PET: state_.secure_pet = static_cast<AutoslideSecurePet>(value); break;
    case AutoslideKey::HOLD_TIME: state_.open_hold_duration = static_cast<uint8_t>(value); break;
    case AutoslideKey::OPEN_FORCE: state_.open_force = static_cast<uint8_t>(value); break;
    case AutoslideKey::CLOSE_FORCE: state_.close_force = static_cast<uint8_t>(value); break;
    case AutoslideKey::CLOSE_END_FORCE: state_.close_end_force = static_cast<uint8_t>(value); break;
    case AutoslideKey::MOTION_STATE: state_.motion_state = static_cast<AutoslideMotionState>(value); break;
    case AutoslideKey::MOTION_TRIGGER: state_.motion_trigger = static_cast<uint8_t>(value); break;
    case AutoslideKey::TRIGGER:
        [[fallthrough]];
    case AutoslideKey::REQUEST_ALL:
        // nothing to do for these two, there is no held internal state for these keys
        ESP_LOGE(TAG, "request all response received successfully");
        break;
    case AutoslideKey::RESULT:
        {
            // for any successful result, set the current state accordingly
            if (value == 1 and inflight_update_ and inflight_update_->key != AutoslideKey::RESULT)
            {
                ESP_LOGV(TAG, "Command successfully set (r:%d).", value);
                update_state(inflight_update_->key, inflight_update_->value);
            }
            else if (value == 0)
            {
                ESP_LOGE(TAG, "Command failed to execute (r:%d).", value);
            }
            break;
        }
    default:
      ESP_LOGW(TAG, "Received unknown key '%c' with value %d", key, value);
      return;
  }
}

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

  if (mode_value != AutoslideMode::UNKNOWN && parent_ != nullptr) {
    ESP_LOGI(TAG, "Queueing mode request: %s", value.c_str());
    parent_->request_set_key_value(AutoslideKey::MODE, mode_value);
  }
  else
  {
    ESP_LOGE(TAG, "Invalid door mode selected or parent not set: %s", value.c_str());
  }
}

void AutoslideSettingNumber::control(float value)
{
  if (parent_ == nullptr || key_ == AutoslideKey::NONE)
  {
    ESP_LOGE(TAG, "Number control missing parent or key");
    return;
  }
  int int_value = (int) (value >= 0 ? value + 0.5f : value - 0.5f); // lroundf without <cmath>

  parent_->request_set_key_value(key_, static_cast<int>(int_value));
  ESP_LOGI(TAG, "Queued setting number (%c): %i", key_, int_value);
}

void AutoslideOnOffSwitch::write_state(bool value)
{
  if (parent_ == nullptr || key_ == AutoslideKey::NONE)
  {
    ESP_LOGE(TAG, "Switch write_state missing parent or key");
    return;
  }

  if (key_ == AutoslideKey::OPEN_SPEED)
  {
      parent_->request_set_key_value(key_, bool_to_speed(value));
  }
  else if (key_ == AutoslideKey::SECURE_PET)
  {
      parent_->request_set_key_value(key_, bool_to_secure_pet(value));
  }
  else
  {
    ESP_LOGE(TAG, "Unknown switch key '%c' in write_state", key_);
  }
}

void AutoslideOpenButton::press_action()
{
  if (parent_ != nullptr)
  {
    parent_->request_set_key_value(AutoslideKey::TRIGGER, AutoslideTrigger::INDOOR);
  }
  else
  {
    ESP_LOGE(TAG, "Open button pressed but parent not set");
  }
}

} // namespace autoslide_door
} // namespace esphome
