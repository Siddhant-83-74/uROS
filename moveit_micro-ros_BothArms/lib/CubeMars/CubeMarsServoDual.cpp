#include "CubeMarsServoDual.h"

// ═══════════════════════════════════════════════════════════════
//  CubeMarsMotor
// ═══════════════════════════════════════════════════════════════

CubeMarsMotor::CubeMarsMotor(uint8_t  motorId,
                             ICanBus &bus,
                             uint8_t  controlMode,
                             int32_t  defaultSpeedErpm,
                             int32_t  defaultAccelErpm2)
  : motor_id(motorId),
    bus_(bus),
    control_mode(controlMode),
    command_can_id(0),
    default_speed_erpm(defaultSpeedErpm),
    default_accel_erpm2(defaultAccelErpm2),
    target_pos_deg(0.0f),
    target_speed_erpm(defaultSpeedErpm),
    target_accel_erpm2(defaultAccelErpm2),
    cur_pos_deg(0.0f),
    cur_speed_erpm(0.0f),
    cur_current_a(0.0f),
    cur_temp_c(0),
    cur_error(0),
    newCommandReceived(false)
{
  updateCanCommandId();
}

void CubeMarsMotor::updateCanCommandId() {
  command_can_id = (static_cast<uint32_t>(control_mode) << 8) | motor_id;
}

void CubeMarsMotor::setTargetPositionDeg(float deg) {
  target_pos_deg     = deg;
  newCommandReceived = true;
}

void CubeMarsMotor::sendPositionSpeedCommand(float   pos_deg,
                                             int32_t speed_erpm,
                                             int32_t accel_erpm2) {
  CAN_message_t msg;
  msg.len            = 8;
  msg.flags.extended = 1;
  msg.flags.remote   = 0;
  msg.id             = command_can_id;
  msg.timestamp      = 0;

  int32_t pos_int = static_cast<int32_t>(pos_deg * 10000.0f);
  int16_t spd_int = static_cast<int16_t>(speed_erpm  / 10);
  int16_t acc_int = static_cast<int16_t>(accel_erpm2 / 10);

  msg.buf[0] = (pos_int >> 24) & 0xFF;
  msg.buf[1] = (pos_int >> 16) & 0xFF;
  msg.buf[2] = (pos_int >>  8) & 0xFF;
  msg.buf[3] =  pos_int        & 0xFF;
  msg.buf[4] = (spd_int >>  8) & 0xFF;
  msg.buf[5] =  spd_int        & 0xFF;
  msg.buf[6] = (acc_int >>  8) & 0xFF;
  msg.buf[7] =  acc_int        & 0xFF;

  bus_.write(msg);

  if (newCommandReceived) {
    Serial.print("[TX] Pos cmd → ID:0x");
    Serial.print(msg.id, HEX);
    Serial.print("  target=");
    Serial.print(pos_deg);
    Serial.println("°");
    newCommandReceived = false;
  }
}

void CubeMarsMotor::update() {
  sendPositionSpeedCommand(target_pos_deg,
                           target_speed_erpm,
                           target_accel_erpm2);
}

void CubeMarsMotor::handleCanMessage(const CAN_message_t &msg) {
  if ((msg.id & 0xFF) != motor_id) return;

  int16_t pos_raw = (static_cast<int16_t>(msg.buf[0]) << 8) | msg.buf[1];
  int16_t spd_raw = (static_cast<int16_t>(msg.buf[2]) << 8) | msg.buf[3];
  int16_t cur_raw = (static_cast<int16_t>(msg.buf[4]) << 8) | msg.buf[5];

  cur_pos_deg    = pos_raw *  0.1f;
  cur_speed_erpm = spd_raw * 10.0f;
  cur_current_a  = cur_raw *  0.01f;

  Serial.printf("[RX] M0x%02X → %.1f°  %.0f ERPM  %.2f A\n",
                motor_id, cur_pos_deg, cur_speed_erpm, cur_current_a);
}

void CubeMarsMotor::setOrigin(uint8_t mode) {
  CAN_message_t msg;
  msg.len            = 1;
  msg.flags.extended = 1;
  msg.flags.remote   = 0;
  msg.id             = (static_cast<uint32_t>(5) << 8) | motor_id;  // CAN_PACKET_SET_ORIGIN_HERE = 5
  msg.timestamp      = 0;
  msg.buf[0]         = mode;
  for (int i = 1; i < 8; i++) msg.buf[i] = 0;

  bus_.write(msg);

  Serial.print("[TX] Origin cmd → ID:0x");
  Serial.print(msg.id, HEX);
  Serial.print("  mode=");
  Serial.println(mode);
}

// ═══════════════════════════════════════════════════════════════
//  Serial command parser
// ═══════════════════════════════════════════════════════════════

void CubeMars_parseSerialCommands(std::vector<CubeMarsMotor> &leftMotors,
                                  std::vector<CubeMarsMotor> &rightMotors) {
  static String buffer;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    buffer += c;

    if (c != '\n' && c != '\r') continue;

    buffer.trim();
    if (buffer.length() == 0) { buffer = ""; continue; }

    // ── Resolve arm prefix: L = left, R = right, none = search both ──
    std::vector<CubeMarsMotor> *onlyLeft  = &leftMotors;
    std::vector<CubeMarsMotor> *onlyRight = &rightMotors;
    bool searchBoth = true;

    char prefix = static_cast<char>(toupper(buffer[0]));
    if (prefix == 'L') {
      buffer      = buffer.substring(1);   // strip prefix
      onlyRight   = nullptr;
      searchBoth  = false;
    } else if (prefix == 'R') {
      buffer      = buffer.substring(1);
      onlyLeft    = nullptr;
      searchBoth  = false;
    }

    // ── Dispatch helper: try one list, return true if handled ──
    auto dispatch = [&](std::vector<CubeMarsMotor> *motors,
                        uint8_t cmd_id,
                        bool    isOrigin,
                        uint8_t mode,
                        float   target) -> bool {
      if (!motors) return false;
      for (auto &motor : *motors) {
        if (motor.getMotorId() == cmd_id) {
          if (isOrigin) motor.setOrigin(mode);
          else          motor.setTargetPositionDeg(target);
          return true;
        }
      }
      return false;
    };

    // ── Parse command ──
    if (buffer.indexOf("origin") > 0) {
      // Format: <hex_id>:origin:<mode>
      int     fc     = buffer.indexOf(':');
      int     lc     = buffer.lastIndexOf(':');
      uint8_t cmd_id = static_cast<uint8_t>(strtol(buffer.substring(0, fc).c_str(), nullptr, 16));
      uint8_t mode   = static_cast<uint8_t>(buffer.substring(lc + 1).toInt());

      if (!dispatch(onlyLeft,  cmd_id, true, mode, 0.0f))
           dispatch(onlyRight, cmd_id, true, mode, 0.0f);

    } else {
      // Format: <hex_id>:<degrees>
      int colon = buffer.indexOf(':');
      if (colon > 0) {
        uint8_t cmd_id = static_cast<uint8_t>(strtol(buffer.substring(0, colon).c_str(), nullptr, 16));
        float   target = buffer.substring(colon + 1).toFloat();

        if (!dispatch(onlyLeft,  cmd_id, false, 0, target))
             dispatch(onlyRight, cmd_id, false, 0, target);
      }
    }

    buffer = "";
  }
}
