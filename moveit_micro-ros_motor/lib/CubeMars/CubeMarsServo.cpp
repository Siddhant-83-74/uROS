#include "CubeMarsServo.h"

// ----------------- Internal state for namespace -----------------

namespace {
  std::vector<CubeMarsMotor> *g_motors = nullptr;

  void canSniff(const CAN_message_t &msg) {
    if (!g_motors) return;
    uint8_t id = msg.id & 0xFF;
    for (auto &motor : *g_motors) {
      if (motor.getMotorId() == id) {
        motor.handleCanMessage(msg);
        return;
      }
    }
  }
}

// ------------- CubeMarsCAN implementation -------------

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CubeMarsCAN::Can0;

void CubeMarsCAN::begin(uint32_t baudrate) {
  Can0.begin();
  Can0.setBaudRate(baudrate);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
}

void CubeMarsCAN::update() {
  // For interrupt-driven receive, nothing is strictly required here.
  // Kept as an extension point if polling is used later.
}

void CubeMarsCAN::attachMotors(std::vector<CubeMarsMotor> &motors) {
  g_motors = &motors;
}

// ------------- CubeMarsMotor implementation -------------

CubeMarsMotor::CubeMarsMotor(uint8_t motorId,
                             uint8_t controlMode,
                             int32_t defaultSpeedErpm,
                             int32_t defaultAccelErpm2)
  : motor_id(motorId),
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
  target_pos_deg = deg;
  newCommandReceived = true;
}

void CubeMarsMotor::sendPositionSpeedCommand(float pos_deg,
                                             int32_t speed_erpm,
                                             int32_t accel_erpm2) {
  CAN_message_t msg;
  msg.len = 8;
  msg.flags.extended = 1;
  msg.flags.remote   = 0;
  msg.id = command_can_id;
  msg.timestamp = 0;

  int32_t pos_int = static_cast<int32_t>(pos_deg * 10000.0f);
  int16_t spd_int = static_cast<int16_t>(speed_erpm / 10);
  int16_t acc_int = static_cast<int16_t>(accel_erpm2 / 10);

  msg.buf[0] = (pos_int >> 24) & 0xFF;
  msg.buf[1] = (pos_int >> 16) & 0xFF;
  msg.buf[2] = (pos_int >> 8)  & 0xFF;
  msg.buf[3] =  pos_int        & 0xFF;
  msg.buf[4] = (spd_int >> 8)  & 0xFF;
  msg.buf[5] =  spd_int        & 0xFF;
  msg.buf[6] = (acc_int >> 8)  & 0xFF;
  msg.buf[7] =  acc_int        & 0xFF;

  CubeMarsCAN::Can0.write(msg);

  if (newCommandReceived) {
    // Serial.print("Pos cmd → ID:0x");
    // Serial.print(msg.id, HEX);
    // Serial.print(" target=");
    // Serial.println(pos_deg);
    newCommandReceived = false;
  }
}

void CubeMarsMotor::update() {
  sendPositionSpeedCommand(target_pos_deg, target_speed_erpm, target_accel_erpm2);
}

void CubeMarsMotor::handleCanMessage(const CAN_message_t &msg) {
  if ((msg.id & 0xFF) == motor_id) {
    int16_t pos_raw = (static_cast<int16_t>(msg.buf[0]) << 8) | msg.buf[1];
    int16_t spd_raw = (static_cast<int16_t>(msg.buf[2]) << 8) | msg.buf[3];
    int16_t cur_raw = (static_cast<int16_t>(msg.buf[4]) << 8) | msg.buf[5];

    cur_pos_deg    = pos_raw * 0.1f;
    cur_speed_erpm = spd_raw * 10.0f;
    cur_current_a  = cur_raw * 0.01f;

    // Serial.printf("M0x%02X: %.1f° %.0f ERPM %.2f A\n",
    //               motor_id, cur_pos_deg, cur_speed_erpm, cur_current_a);
  }
}

void CubeMarsMotor::setOrigin(uint8_t mode) {
  CAN_message_t msg;
  msg.len = 1;
  msg.flags.extended = 1;
  msg.flags.remote   = 0;
  // CAN_PACKET_SET_ORIGIN_HERE = 5
  msg.id = (static_cast<uint32_t>(5) << 8) | motor_id;
  msg.timestamp = 0;
  msg.buf[0] = mode;
  for (int i = 1; i < 8; i++) msg.buf[i] = 0;

  CubeMarsCAN::Can0.write(msg);

  Serial.print("Origin cmd → ID:0x");
  Serial.print(msg.id, HEX);
  Serial.print(" mode=");
  Serial.println(mode);
}

// ------------- Serial command parser -------------

void CubeMars_parseSerialCommands(std::vector<CubeMarsMotor> &motors) {
  static String buffer;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    buffer += c;

    if (c == '\n' || c == '\r') {
      buffer.trim();
      if (buffer.length() == 0) { buffer = ""; continue; }

      int origin_pos = buffer.indexOf("origin");
      if (origin_pos > 0) {
        // "03:origin:1"
        int first_colon = buffer.indexOf(':');
        int last_colon  = buffer.lastIndexOf(':');
        String id_str   = buffer.substring(0, first_colon);
        String mode_str = buffer.substring(last_colon + 1);

        uint8_t cmd_id = static_cast<uint8_t>(strtol(id_str.c_str(), nullptr, 16));
        uint8_t mode   = static_cast<uint8_t>(mode_str.toInt());

        for (auto &motor : motors) {
          if (motor.getMotorId() == cmd_id) {
            motor.setOrigin(mode);
            break;
          }
        }
      } else {
        // "03:360"
        int colon = buffer.indexOf(':');
        if (colon > 0) {
          String id_str  = buffer.substring(0, colon);
          String pos_str = buffer.substring(colon + 1);

          uint8_t cmd_id = static_cast<uint8_t>(strtol(id_str.c_str(), nullptr, 16));
          float   target = pos_str.toFloat();

          for (auto &motor : motors) {
            if (motor.getMotorId() == cmd_id) {
              motor.setTargetPositionDeg(target);
              break;
            }
          }
        }
      }

      buffer = "";
    }
  }
}
