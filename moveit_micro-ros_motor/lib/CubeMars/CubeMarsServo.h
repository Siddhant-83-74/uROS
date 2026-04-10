#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>

/**
 * @brief Class to control a single CubeMars motor over CAN using FlexCAN_T4.
 *
 * Typical usage:
 * @code
 * #include <CubeMarsServo.h>
 *
 * CubeMarsMotor motor(0x03);  // 0x03 = motor CAN ID (LSB)
 *
 * void setup() {
 *   CubeMarsCAN::begin(1000000); // 1 Mbps CAN
 *   motor.setOrigin(0);          // temporary origin
 * }
 *
 * void loop() {
 *   motor.setTargetPositionDeg(90.0f);
 *   motor.update();
 *   delay(20);
 * }
 * @endcode
 */
class CubeMarsMotor {
public:
  /**
   * @brief Construct a new CubeMarsMotor.
   *
   * @param motorId       8-bit motor CAN ID (LSB of extended ID).
   * @param controlMode   CubeMars control mode (default 6 = position/speed).
   * @param defaultSpeedErpm   Default speed limit in eRPM.
   * @param defaultAccelErpm2  Default acceleration limit in eRPM/s².
   */
  CubeMarsMotor(uint8_t motorId,
                uint8_t controlMode = 6,
                int32_t defaultSpeedErpm = 5000,
                int32_t defaultAccelErpm2 = 8000);

  /**
   * @brief Send the latest target position/speed/accel command to the motor.
   *
   * Call this periodically (e.g. every 10–20 ms).
   */
  void update();

  /// @brief Get motor CAN ID (LSB).
  uint8_t getMotorId() const { return motor_id; }

  /// @brief Get current position in degrees (decoded from feedback frame).
  float getCurrentPosDeg() const { return cur_pos_deg; }

  /**
   * @brief Set the target position in degrees.
   *
   * @param deg Target position in mechanical degrees.
   */
  void setTargetPositionDeg(float deg);

  /**
   * @brief Set the origin of the encoder.
   *
   * @param mode 0 = temporary, 1 = permanent, 2 = restore default.
   */
  void setOrigin(uint8_t mode);

  /**
   * @brief Update internal feedback state from an incoming CAN frame.
   *
   * Call this from the global CAN receive callback.
   *
   * @param msg Incoming CAN message.
   */
  void handleCanMessage(const CAN_message_t &msg);

private:
  uint8_t  motor_id, control_mode;
  uint32_t command_can_id;
  int32_t  default_speed_erpm, default_accel_erpm2;

  float    target_pos_deg;
  int32_t  target_speed_erpm, target_accel_erpm2;

  float    cur_pos_deg, cur_speed_erpm, cur_current_a;
  int8_t   cur_temp_c;
  uint8_t  cur_error;
  bool     newCommandReceived;

  void updateCanCommandId();
  void sendPositionSpeedCommand(float pos_deg, int32_t speed_erpm, int32_t accel_erpm2);
};

/**
 * @brief Simple namespace wrapper around the FlexCAN_T4 object and helpers.
 *
 * This keeps the global CAN object inside the library while still letting
 * you access it when needed (for filters, etc.).
 */
namespace CubeMarsCAN {

  /// Global CAN object for CAN1 on Teensy 4.x (1 Mbps typical).[web:21][web:18]
  extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

  /**
   * @brief Initialize the CAN bus and attach the receive callback.
   *
   * @param baudrate CAN bit rate in bit/s (e.g. 1000000 for 1 Mbps).
   */
  void begin(uint32_t baudrate = 1000000);

  /**
   * @brief Poll CAN, forwarding any received messages to motors.
   *
   * Call this in loop() if you are not using interrupts,
   * or leave as-is if using `onReceive()`.
   */
  void update();

  /**
   * @brief Register a list of motors so the CAN callback can dispatch frames.
   *
   * @param motors Reference to a std::vector of CubeMarsMotor.
   */
  void attachMotors(std::vector<CubeMarsMotor> &motors);

} // namespace CubeMarsCAN

/**
 * @brief Parse serial commands and update motors.
 *
 * Format:
 *  - Position: "03:360"       → motor 0x03 to 360 degrees
 *  - Origin:   "03:origin:1"  → set origin mode 1 on motor 0x03
 *
 * @param motors Vector of motors to be controlled.
 */
void CubeMars_parseSerialCommands(std::vector<CubeMarsMotor> &motors);
