#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>

// ═══════════════════════════════════════════════════════════════
//  Abstract CAN bus interface
//  CubeMarsMotor talks only to this — fully bus-agnostic
// ═══════════════════════════════════════════════════════════════
class ICanBus {
public:
  virtual void write(const CAN_message_t &msg) = 0;
  virtual ~ICanBus() = default;
};

// Forward declaration
class CubeMarsMotor;

// ═══════════════════════════════════════════════════════════════
//  CubeMarsCAN<BusNum>
//  One instance per physical CAN bus (CAN1 or CAN3)
//
//  Template statics are scoped per instantiation, so
//  CubeMarsCAN<CAN1> and CubeMarsCAN<CAN3> each own their own
//  FlexCAN driver and their own motor list pointer — zero
//  cross-bus interference, zero runtime branching.
// ═══════════════════════════════════════════════════════════════
template <CAN_DEV_TABLE BusNum>
class CubeMarsCAN : public ICanBus {
public:
  void begin(uint32_t baudrate = 1000000);
  void update();                                        // extension point
  void attachMotors(std::vector<CubeMarsMotor> &motors);
  void write(const CAN_message_t &msg) override;

  // Raw driver — accessible if direct FlexCAN calls are needed
  static FlexCAN_T4<BusNum, RX_SIZE_256, TX_SIZE_16> Can;

private:
  static std::vector<CubeMarsMotor> *s_motors;
  static void canSniff(const CAN_message_t &msg);
};

// ═══════════════════════════════════════════════════════════════
//  CubeMarsMotor
// ═══════════════════════════════════════════════════════════════
class CubeMarsMotor {
public:
  // bus  — pass the CubeMarsCAN<CAN1> or CubeMarsCAN<CAN3> instance
  CubeMarsMotor(uint8_t  motorId,
                ICanBus &bus,
                uint8_t  controlMode       = 6,
                int32_t  defaultSpeedErpm  = 60000,
                int32_t  defaultAccelErpm2 = 200000);

  // ── Commands ──────────────────────────────────────────────
  void setTargetPositionDeg(float deg);
  void setOrigin(uint8_t mode = 0);
  void update();   // call every control-loop tick to push CAN frame

  // ── Setters for speed / accel overrides ──────────────────
  void setSpeedErpm(int32_t erpm)   { target_speed_erpm  = erpm;  }
  void setAccelErpm2(int32_t erpm2) { target_accel_erpm2 = erpm2; }

  // ── Feedback getters ──────────────────────────────────────
  // Values are updated every time the motor sends a CAN feedback packet.
  float   getCurPosDeg()    const { return cur_pos_deg;    }  // degrees
  float   getCurSpeedErpm() const { return cur_speed_erpm; }  // electrical RPM
  float   getCurCurrentA()  const { return cur_current_a;  }  // amps
  uint8_t getTempC()        const { return cur_temp_c;     }  // driver board temp °C
  uint8_t getMotorId()      const { return motor_id;       }

  // Error code from motor feedback (manual §5.2):
  //   0 = no fault
  //   1 = over-temperature
  //   2 = over-current
  //   3 = over-voltage
  //   4 = under-voltage
  //   5 = encoder fault
  //   6 = phase current unbalance (hardware likely damaged)
  uint8_t getErrorCode()    const { return cur_error;      }

  // ── Readiness check ───────────────────────────────────────
  // Returns true once this motor has sent at least one CAN feedback
  // packet. Use this to confirm the motor is powered and communicating
  // BEFORE issuing position commands — prevents dangerous startup jerks
  // when Teensy boots before motor power is applied.
  bool hasFeedback() const { return feedbackReceived; }
  void clearFeedback() { feedbackReceived = false; }

  // Called by canSniff — not for user code
  void handleCanMessage(const CAN_message_t &msg);

private:
  void updateCanCommandId();
  void sendPositionSpeedCommand(float pos_deg,
                                int32_t speed_erpm,
                                int32_t accel_erpm2);

  uint8_t   motor_id;
  uint8_t   control_mode;
  uint32_t  command_can_id;
  int32_t   default_speed_erpm;
  int32_t   default_accel_erpm2;

  float     target_pos_deg;
  int32_t   target_speed_erpm;
  int32_t   target_accel_erpm2;

  float     cur_pos_deg;
  float     cur_speed_erpm;
  float     cur_current_a;
  uint8_t   cur_temp_c;
  uint8_t   cur_error;

  bool      newCommandReceived;
  bool      feedbackReceived;   // set true on first CAN feedback packet
  ICanBus  &bus_;
};

// ═══════════════════════════════════════════════════════════════
//  Serial command parser
//  Searches both arm lists; left arm has priority on ID collision
//
//  Supported formats:
//    Position  →  <hex_id>:<degrees>          e.g.  03:180.0
//    Origin    →  <hex_id>:origin:<mode>      e.g.  03:origin:0
//    Arm-prefix (optional, for same-ID motors on both buses):
//                 L<hex_id>:<degrees>         e.g.  L03:180.0
//                 R<hex_id>:origin:0          e.g.  R03:origin:0
// ═══════════════════════════════════════════════════════════════
void CubeMars_parseSerialCommands(std::vector<CubeMarsMotor> &leftMotors,
                                  std::vector<CubeMarsMotor> &rightMotors);

// ═══════════════════════════════════════════════════════════════
//  Template definitions  (must live in the header)
// ═══════════════════════════════════════════════════════════════

template <CAN_DEV_TABLE BusNum>
FlexCAN_T4<BusNum, RX_SIZE_256, TX_SIZE_16> CubeMarsCAN<BusNum>::Can;

template <CAN_DEV_TABLE BusNum>
std::vector<CubeMarsMotor> *CubeMarsCAN<BusNum>::s_motors = nullptr;

template <CAN_DEV_TABLE BusNum>
void CubeMarsCAN<BusNum>::begin(uint32_t baudrate) {
  Can.begin();
  Can.setBaudRate(baudrate);
  Can.enableFIFO();
  Can.enableFIFOInterrupt();
  Can.onReceive(canSniff);
}

template <CAN_DEV_TABLE BusNum>
void CubeMarsCAN<BusNum>::update() {
  // Polling extension point — nothing needed for interrupt-driven mode
}

template <CAN_DEV_TABLE BusNum>
void CubeMarsCAN<BusNum>::attachMotors(std::vector<CubeMarsMotor> &motors) {
  s_motors = &motors;
}

template <CAN_DEV_TABLE BusNum>
void CubeMarsCAN<BusNum>::write(const CAN_message_t &msg) {
  Can.write(msg);
}

template <CAN_DEV_TABLE BusNum>
void CubeMarsCAN<BusNum>::canSniff(const CAN_message_t &msg) {
  if (!s_motors) return;
  uint8_t id = msg.id & 0xFF;
  for (auto &motor : *s_motors) {
    if (motor.getMotorId() == id) {
      motor.handleCanMessage(msg);
      return;
    }
  }
}
