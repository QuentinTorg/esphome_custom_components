#include "autoslide_door.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace autoslide_door {

// Helper function to convert a float setting value to a zero-padded string (e.g., 0 -> "00", 15 -> "15")
// Used for the Open Hold Time ('j') command which expects two digits.
std::string format_setting_value(float value) {
  int val = static_cast<int>(value);
  if (val < 10) {
    return "0" + to_string(val);
  }
  return to_string(val);
}

// ====================================================================
// --- Main AutoslideDoor Component Implementation ---
// ====================================================================

void AutoslideDoor::send_command(const std::string &command, uint32_t wait_ms) {
  if (this->waiting_for_result_) {
    ESP_LOGW(TAG, "Skipping command '%s', already waiting for previous result.", command.c_str());
    return;
  }

  // Format the command: AT+UPDATE,<command>\e
  std::string full_command = "AT+UPDATE," + command + (char) END_CHAR;

  ESP_LOGD(TAG, "Sending command: %s", full_command.c_str());
  this->write_str(full_command.c_str());

  // Set state to indicate we are waiting for a reply
  this->last_command_time_ = millis();
  this->waiting_for_result_ = true;
}

void AutoslideDoor::request_status() {
  // Command to request current settings: AT+UPDATE,d:0\e
  this->send_command("d:0", 100);
}

// Utility function to split a string into key-value pairs (e.g., "a:0,e:1" -> {"a":"0", "e":"1"})
std::map<std::string, std::string> AutoslideDoor::parse_key_values(const std::string &payload) {
  std::map<std::string, std::string> key_values;

  // Split the payload string by comma (,)
  std::vector<std::string> parts = split(payload, ',');

  for (const auto &part : parts) {
    // Split each part by colon (:)
    size_t colon_pos = part.find(':');
    if (colon_pos != std::string::npos) {
      std::string key = part.substr(0, colon_pos);
      std::string value = part.substr(colon_pos + 1);
      key_values[key] = value;
    }
  }
  return key_values;
}


void AutoslideDoor::parse_status(const std::string &payload) {
  // The Autoslide can send:
  // 1. AT+RESULT,r:1 (Success reply to AT+UPDATE)
  // 2. AT+RESULT,a:0,e:1,... (Full status reply to AT+UPDATE,d:0\e)
  // 3. AT+UPSEND,m:2,c:1,... (Unprompted status update)

  // Only stop waiting if the response is AT+RESULT
  if (payload.rfind("RESULT,", 0) == 0) {
      this->waiting_for_result_ = false;
  }

  // Find the command type and content
  size_t comma_pos = payload.find(',');
  if (comma_pos == std::string::npos) {
    ESP_LOGW(TAG, "Invalid Autoslide response format: %s", payload.c_str());
    return;
  }

  std::string command_type = payload.substr(0, comma_pos);
  std::string content = payload.substr(comma_pos + 1);

  ESP_LOGD(TAG, "Received payload content: %s", content.c_str());

  // Check if it's a simple success reply "r:1"
  if (content == "r:1" || content == "r:0") {
      ESP_LOGI(TAG, "Received command result: %s", content.c_str());
      return;
  }

  // Parse key-value settings from the content (e.g., "a:0,e:1,...")
  std::map<std::string, std::string> settings = this->parse_key_values(content);

  // --- Update All Entities ---

  // Key 'a': Door Mode (Select)
  if (settings.count("a") && this->mode_select_ != nullptr) {
    int mode_value = safe_str_to_int(settings.at("a")).value_or(-1);
    if (mode_value != -1) {
      for (const auto &pair : this->mode_select_->value_map_) {
        if (pair.second == mode_value) {
          if (this->mode_select_->get_state() != pair.first) {
            this->mode_select_->publish_state(pair.first);
            ESP_LOGV(TAG, "Updated Door Mode (a) to: %s", pair.first.c_str());
          }
          break;
        }
      }
    }
  }

  // Key 'e': Open Speed (Switch)
  if (settings.count("e") && this->open_speed_switch_ != nullptr) {
    bool state = settings.at("e") == "1";
    if (this->open_speed_switch_->state != state) {
      this->open_speed_switch_->publish_state(state);
      ESP_LOGV(TAG, "Updated Open Speed (e) to: %s", state ? "ON" : "OFF");
    }
  }

  // Key 'g': Secure Pet Mode (Switch)
  if (settings.count("g") && this->secure_pet_switch_ != nullptr) {
    bool state = settings.at("g") == "1";
    if (this->secure_pet_switch_->state != state) {
      this->secure_pet_switch_->publish_state(state);
      ESP_LOGV(TAG, "Updated Secure Pet Mode (g) to: %s", state ? "ON" : "OFF");
    }
  }

  // Key 'j': Open Hold Duration (Number)
  if (settings.count("j") && this->open_hold_number_ != nullptr) {
    // Settings for 'j' come as a zero-padded string (e.g., "05"), convert to int
    float value = safe_str_to_int(settings.at("j")).value_or(0);
    if (this->open_hold_number_->state != value) {
      this->open_hold_number_->publish_state(value);
      ESP_LOGV(TAG, "Updated Open Hold Duration (j) to: %.0f", value);
    }
  }

  // Key 'C': Open Force (Number)
  if (settings.count("C") && this->open_open_force_number_ != nullptr) {
    float value = safe_str_to_int(settings.at("C")).value_or(0);
    if (this->open_open_force_number_->state != value) {
      this->open_open_force_number_->publish_state(value);
      ESP_LOGV(TAG, "Updated Open Force (C) to: %.0f", value);
    }
  }

  // Key 'Z': Close Force (Number)
  if (settings.count("Z") && this->close_force_number_ != nullptr) {
    float value = safe_str_to_int(settings.at("Z")).value_or(0);
    if (this->close_force_number_->state != value) {
      this->close_force_number_->publish_state(value);
      ESP_LOGV(TAG, "Updated Close Force (Z) to: %.0f", value);
    }
  }

  // Key 'A': Close End Force (Number)
  if (settings.count("A") && this->close_end_force_number_ != nullptr) {
    float value = safe_str_to_int(settings.at("A")).value_or(0);
    if (this->close_end_force_number_->state != value) {
      this->close_end_force_number_->publish_state(value);
      ESP_LOGV(TAG, "Updated Close End Force (A) to: %.0f", value);
    }
  }

  // --- Update Read-only Text Sensors ---

  // Key 'm': Motion State (Text Sensor)
  if (settings.count("m") && this->motion_state_sensor_ != nullptr) {
    std::string motion_state;
    // m:2 means Motion Detected, m:0 means No Motion (inferred)
    if (settings.at("m") == "2") {
      motion_state = "Motion Detected";
    } else if (settings.at("m") == "0") {
      motion_state = "No Motion";
    } else {
      motion_state = "Unknown (" + settings.at("m") + ")";
    }
    if (this->motion_state_sensor_->get_state() != motion_state) {
      this->motion_state_sensor_->publish_state(motion_state);
      ESP_LOGV(TAG, "Updated Motion State (m) to: %s", motion_state.c_str());
    }
  }

  // Key 'c': Lock State (Text Sensor)
  if (settings.count("c") && this->lock_state_sensor_ != nullptr) {
    std::string lock_state;
    // c:1 means Locked, c:0 means Unlocked
    if (settings.at("c") == "1") {
      lock_state = "Locked";
    } else if (settings.at("c") == "0") {
      lock_state = "Unlocked";
    } else {
      lock_state = "Unknown (" + settings.at("c") + ")";
    }
    if (this->lock_state_sensor_->get_state() != lock_state) {
      this->lock_state_sensor_->publish_state(lock_state);
      ESP_LOGV(TAG, "Updated Lock State (c) to: %s", lock_state.c_str());
    }
  }
}

// --- ESPHome Component Overrides ---

void AutoslideDoor::setup() {
  // Set all entities to point back to this parent component
  if (this->trigger_button_) this->trigger_button_->set_parent(this);
  if (this->mode_select_) this->mode_select_->set_parent(this);
  if (this->open_speed_switch_) this->open_speed_switch_->set_parent(this);
  if (this->secure_pet_switch_) this->secure_pet_switch_->set_parent(this);
  if (this->open_hold_number_) this->open_hold_number_->set_parent(this);
  if (this->open_open_force_number_) this->open_open_force_number_->set_parent(this);
  if (this->close_force_number_) this->close_force_number_->set_parent(this);
  if (this->close_end_force_number_) this->close_end_force_number_->set_parent(this);
  if (this->motion_state_sensor_) this->motion_state_sensor_->set_parent(this);
  if (this->lock_state_sensor_) this->lock_state_sensor_->set_parent(this);

  ESP_LOGCONFIG(TAG, "Autoslide Door Component initialized.");
  // Request initial status immediately after setup
  this->request_status();
}

void AutoslideDoor::update() {
  // This is called periodically based on the 'update_interval' in YAML
  // Use this to proactively request the door's current status if no update has been received recently
  this->request_status();
}

void AutoslideDoor::loop() {
  // Check for incoming data from the UART bus
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);

    if (byte == END_CHAR) {
      // End of command detected
      if (this->receive_buffer_.length() > 0) {
        // Full command is AT+<TYPE>,<PAYLOAD> (without the trailing \e)
        // We expect the beginning "AT+" followed by the command type.
        if (this->receive_buffer_.rfind("AT+", 0) == 0) {
          // Remove "AT+" prefix
          std::string payload = this->receive_buffer_.substr(3);
          this->parse_status(payload);
          this->last_status_update_time_ = millis();
        } else {
          ESP_LOGW(TAG, "Received malformed start of command: %s", this->receive_buffer_.c_str());
        }
      }
      // Reset buffer regardless of success
      this->receive_buffer_.clear();
    } else {
      // Append byte to the buffer
      this->receive_buffer_ += (char) byte;
    }
  }

  // Timeout check for commands sent
  // If we sent a command and haven't received a result after 500ms, assume failure.
  if (this->waiting_for_result_ && (millis() - this->last_command_time_ > 500)) {
    ESP_LOGW(TAG, "Command timeout. No AT+RESULT received.");
    this->waiting_for_result_ = false;
  }
}

// ====================================================================
// --- Entity Command Implementations (Called by entity callbacks) ---
// ====================================================================

// Button (Master Trigger: b:0)
void AutoslideDoor::trigger_master() {
  this->send_command("b:0");
}

// Select (Door Mode: a:<mode>)
void AutoslideDoor::set_door_mode(int mode) {
  this->send_command("a:" + to_string(mode));
}

// Switch (Open Speed: e:1/0)
void AutoslideDoor::set_open_speed(bool state) {
  this->send_command("e:" + to_string(state ? 1 : 0));
}

// Switch (Secure Pet Mode: g:1/0)
void AutoslideDoor::set_secure_pet(bool state) {
  this->send_command("g:" + to_string(state ? 1 : 0));
}

// Number (Open Hold Time: j:<time>)
void AutoslideDoor::set_open_hold_time(float value) {
  this->send_command("j:" + format_setting_value(value));
}

// Number (Open Force: C:<force>)
void AutoslideDoor::set_open_force(float value) {
  this->send_command("C:" + to_string(static_cast<int>(value)));
}

// Number (Close Force: Z:<force>)
void AutoslideDoor::set_close_force(float value) {
  this->send_command("Z:" + to_string(static_cast<int>(value)));
}

// Number (Close End Force: A:<force>)
void AutoslideDoor::set_close_end_force(float value) {
  this->send_command("A:" + to_string(static_cast<int>(value)));
}

// ====================================================================
// --- Entity Callback Overrides (Called by Home Assistant / API) ---
// ====================================================================

// Button
void AutoslideTriggerButton::press_action() {
  this->parent_->trigger_master();
}

// Select
void AutoslideModeSelect::control(const std::string &value) {
  if (this->value_map_.count(value)) {
    this->parent_->set_door_mode(this->value_map_.at(value));
    // Publish pending state while waiting for confirmation from Autoslide
    this->publish_state(value);
  }
}

// Switch
void AutoslideOpenSpeedSwitch::write_state(bool state) {
  this->parent_->set_open_speed(state);
  this->publish_state(state);
}

void AutoslideSecurePetSwitch::write_state(bool state) {
  this->parent_->set_secure_pet(state);
  this->publish_state(state);
}

// Number
void AutoslideOpenHoldNumber::control(float value) {
  this->parent_->set_open_hold_time(value);
  this->publish_state(value);
}

void AutoslideOpenForceNumber::control(float value) {
  this->parent_->set_open_force(value);
  this->publish_state(value);
}

void AutoslideCloseForceNumber::control(float value) {
  this->parent_->set_close_force(value);
  this->publish_state(value);
}

void AutoslideCloseEndForceNumber::control(float value) {
  this->parent_->set_close_end_force(value);
  this->publish_state(value);
}

} // namespace autoslide_door
} // namespace esphome
