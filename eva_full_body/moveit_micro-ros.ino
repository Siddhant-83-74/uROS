#include "CubeMarsServoDual.h"
#include <Wire.h>
#include "RMCS220xTeensy.h"
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int16.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  Joint indices — 12 DOF, both arms
//
//  [0]  Left shoulder pitch   (CAN1  0x01)
//  [1]  Left shoulder roll    (CAN1  0x02)
//  [2]  Left bicep            (CAN1  0x03)
//  [3]  Left elbow            (CAN1  0x04)
//  [4]  Left forearm           (RMCS_Left,  Wire,  0x08)
//  [5]  Left wrist             (Servo, pin 10)
//  [6]  Right shoulder pitch  (CAN3  0x05)
//  [7]  Right shoulder roll   (CAN3  0x06)
//  [8]  Right bicep           (CAN3  0x07)
//  [9]  Right elbow           (CAN3  0x08)
//  [10] Right forearm          (RMCS_Right, Wire1, 0x08)
//  [11] Right wrist            (Servo, pin 1)
// ═══════════════════════════════════════════════════════════════════════════════

#define NUM_JOINTS 12

// ── Arm hardware config ──────────────────────────────────────────────────────
#define RMCS_ADDR       0x08
#define SERVO_PIN_LEFT  10
#define SERVO_PIN_RIGHT 1

// ── CAN bus instances ────────────────────────────────────────────────────────
CubeMarsCAN<CAN1> leftBus;
CubeMarsCAN<CAN3> rightBus;

std::vector<CubeMarsMotor> leftArm = {
  CubeMarsMotor(0x01, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x02, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x03, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x04, leftBus, 6, 4000, 6000),
};

std::vector<CubeMarsMotor> rightArm = {
  CubeMarsMotor(0x05, rightBus, 6, 6000, 8000),
  CubeMarsMotor(0x06, rightBus, 6, 6000, 8000),
  CubeMarsMotor(0x01, rightBus, 6, 6000, 8000),
  CubeMarsMotor(0x02, rightBus, 6, 6000, 8000),
};
bool g_originSet = false;

// ── RMCS motors ──────────────────────────────────────────────────────────────
// Joint 4  (left forearm)  : Wire1  = pins 17 SDA / 16 SCL
// Joint 10 (right forearm) : Wire   = pins 19 SDA / 18 SCL
RMCS220xTeensy rmcsLeft(RMCS_ADDR, Wire1);
RMCS220xTeensy rmcsRight(RMCS_ADDR, Wire);

// ── Wrist servos ─────────────────────────────────────────────────────────────
Servo wristLeftServo;
Servo wristRightServo;

// ═══════════════════════════════════════════════════════════════════════════════
//  System State Machine (guards arm hardware)
//
//  SYS_WAITING    → polling for CAN feedback, zero motor commands
//  SYS_MONITORING → showing raw encoder angles, waiting for operator key-press
//  SYS_READY      → full arm operation active
//
//  Head animatronics are NOT gated — they respond to /talk_state and
//  /neck_angle immediately after micro-ROS connects.
// ═══════════════════════════════════════════════════════════════════════════════

enum SysState { SYS_WAITING, SYS_HOMING, SYS_MONITORING, SYS_READY };
SysState g_sysState = SYS_WAITING;

#define MOTOR_WAIT_TIMEOUT_MS 1000000
uint32_t g_waitStartMs = 0;

// ── Soft position limits (degrees) ───────────────────────────────────────────
const float JOINT_MIN[NUM_JOINTS] = { -360, -360, -360, -360, -360, -360,
                                       -360, -360, -360, -360, -360, -360 };
const float JOINT_MAX[NUM_JOINTS] = {  360,  360,  360,  360,  360,  360,
                                        360,  360,  360,  360,  360,  360 };

// ═══════════════════════════════════════════════════════════════════════════════
//  Gesture engine
// ═══════════════════════════════════════════════════════════════════════════════

#define MOTOR_DEFAULT_SPEED_ERPM 4000

struct Keyframe {
  float    joints[NUM_JOINTS];
  uint32_t duration_ms;
  int32_t  speed_erpm;
};

#define MAX_KEYFRAMES 20

struct Gesture {
  Keyframe frames[MAX_KEYFRAMES];
  uint8_t  count;
};

float g_jointTargets[NUM_JOINTS] = {0};

float g_prevRmcsLeft   = -999.0f;
float g_prevRmcsRight  = -999.0f;
float g_prevServoLeft  = -999.0f;
float g_prevServoRight = -999.0f;

enum GestureState { GESTURE_IDLE, GESTURE_PLAYING };
GestureState g_gestureState = GESTURE_IDLE;

const Gesture *g_currentGesture = nullptr;
uint8_t  g_currentFrame   = 0;
float    g_frameStart[NUM_JOINTS] = {0};
uint32_t g_frameStartMs   = 0;

float g_tangents[MAX_KEYFRAMES + 1][NUM_JOINTS];

// ── Predefined gestures ──────────────────────────────────────────────────────

const Gesture GESTURE_HOME_SLOW = {
  .frames = {
    { .joints = {0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0}, .duration_ms = 3000, .speed_erpm = 1000 },
  },
  .count = 1,
};

const Gesture GESTURE_HOME = {
  .frames = {
    { .joints = {0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0}, .duration_ms = 1000, .speed_erpm = 3000 },
  },
  .count = 1,
};

const Gesture GESTURE_HELLO = {
  .frames = {
    { .joints = {0, 0, 0, 0, 0, 0,   48, 3, 0, -107, 91, 35},     .duration_ms = 5000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 3, -8, -107, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 3, 22, -107, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 3,-13, -107, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 3, 26, -107, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 3, 13, -107, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 2, 13, -107, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   48, 2, 10,  -70, 91, 35},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,   24, 2,  6,  -50, 45, 20},    .duration_ms = 1000, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,    0, 0,  0,    0,  0,  0},    .duration_ms = 1000, .speed_erpm = 3000 },
  },
  .count = 10,
};

const Gesture GESTURE_BYE = {
  .frames = {
    { .joints = {-20, 15, 0, -30, 0, 0,   -20, 15, 0, -30, 0, 0},         .duration_ms = 800, .speed_erpm = 3000 },
    { .joints = {-20, 25, 0, -30, 10, 15,   -20, 25, 0, -30, 10, 15},     .duration_ms = 400, .speed_erpm = 5000 },
    { .joints = {-20, 15, 0, -30, -10, -15, -20, 15, 0, -30, -10, -15},   .duration_ms = 400, .speed_erpm = 5000 },
    { .joints = {-20, 25, 0, -30, 10, 15,   -20, 25, 0, -30, 10, 15},     .duration_ms = 400, .speed_erpm = 5000 },
    { .joints = {0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0},           .duration_ms = 800, .speed_erpm = 3000 },
  },
  .count = 5,
};

const Gesture GESTURE_NAMASTE = {
  .frames = {
    { .joints = {-20, -15, 0, -60, 0, 0,   -20, -15, 0, -60, 0, 0}, .duration_ms = 1000, .speed_erpm = 2000 },
    { .joints = {-20, -15, 0, -60, 0, 0,   -20, -15, 0, -60, 0, 0}, .duration_ms = 1000, .speed_erpm = 2000 },
    { .joints = {0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0},     .duration_ms = 1000, .speed_erpm = 2000 },
  },
  .count = 3,
};

const Gesture GESTURE_HANDSHAKE = {
  .frames = {
    { .joints = {0, 0, 0, 0, 0, 0,   3, 4,   0,   -10, 0, 0}, .duration_ms = 1000, .speed_erpm = 2000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5,   0,   -50, 0, 0}, .duration_ms = 1000, .speed_erpm = 3000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -76, 0, 0}, .duration_ms = 2000, .speed_erpm = 3000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -76, 0, 0}, .duration_ms = 1000, .speed_erpm = 7000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -70, 0, 0}, .duration_ms = 1000, .speed_erpm = 7000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -76, 0, 0}, .duration_ms = 1000, .speed_erpm = 7000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -70, 0, 0}, .duration_ms = 1000, .speed_erpm = 7000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -70, 0, 0}, .duration_ms = 1000, .speed_erpm = 7000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 5, -11,   -50, 0, 0}, .duration_ms = 1000, .speed_erpm = 3000 },
    { .joints = {0, 0, 0, 0, 0, 0,   3, 4,   0,     0, 0, 0}, .duration_ms = 1000, .speed_erpm = 2000 },
    { .joints = {0, 0, 0, 0, 0, 0,   0, 0,   0,     0, 0, 0}, .duration_ms = 1000, .speed_erpm = 2000 },
  },
  .count = 11,
};

// ── Catmull-Rom tangent computation ──────────────────────────────────────────
void computeTangents(const Gesture *g, const float startPos[NUM_JOINTS]) {
  int n = g->count;
  for (int j = 0; j < NUM_JOINTS; j++) g_tangents[0][j] = 0.0f;
  for (int j = 0; j < NUM_JOINTS; j++) g_tangents[n][j] = 0.0f;

  for (int i = 1; i < n; i++) {
    float dt_prev = g->frames[i-1].duration_ms / 1000.0f;
    float dt_next = g->frames[i].duration_ms   / 1000.0f;

    const float *p_prev = (i == 1) ? startPos         : g->frames[i-2].joints;
    const float *p_cur  =                               g->frames[i-1].joints;
    const float *p_next =                               g->frames[i].joints;

    for (int j = 0; j < NUM_JOINTS; j++) {
      float slope_in  = (p_cur[j]  - p_prev[j]) / dt_prev;
      float slope_out = (p_next[j] - p_cur[j])  / dt_next;
      g_tangents[i][j] = 0.5f * (slope_in + slope_out);
    }
  }
}

void applyFrameSpeed(const Keyframe &frame) {
  int32_t spd = (frame.speed_erpm > 0) ? frame.speed_erpm : MOTOR_DEFAULT_SPEED_ERPM;
  for (auto &m : leftArm)  m.setSpeedErpm(spd);
  for (auto &m : rightArm) m.setSpeedErpm(spd);
}

void startGesture(const Gesture *gesture) {
  g_currentGesture = gesture;
  g_currentFrame   = 0;
  g_gestureState   = GESTURE_PLAYING;
  g_frameStartMs   = millis();
  for (int i = 0; i < NUM_JOINTS; i++) g_frameStart[i] = g_jointTargets[i];
  computeTangents(gesture, g_frameStart);
  applyFrameSpeed(gesture->frames[0]);
  Serial.print("[GESTURE] Started, frames=");
  Serial.println(gesture->count);
}

void updateGesture() {
  if (g_gestureState != GESTURE_PLAYING || !g_currentGesture) return;

  const Keyframe &target = g_currentGesture->frames[g_currentFrame];
  uint32_t elapsed = millis() - g_frameStartMs;

  float t = (target.duration_ms > 0)
            ? (float)elapsed / (float)target.duration_ms
            : 1.0f;
  if (t > 1.0f) t = 1.0f;

  float t2  = t * t;
  float t3  = t2 * t;
  float h00 =  2.0f*t3 - 3.0f*t2 + 1.0f;
  float h10 =       t3 - 2.0f*t2 + t;
  float h01 = -2.0f*t3 + 3.0f*t2;
  float h11 =       t3 -       t2;
  float dt  = target.duration_ms / 1000.0f;

  for (int i = 0; i < NUM_JOINTS; i++) {
    float v0 = g_tangents[g_currentFrame][i];
    float v1 = g_tangents[g_currentFrame + 1][i];
    g_jointTargets[i] = h00 * g_frameStart[i]
                      + h10 * dt * v0
                      + h01 * target.joints[i]
                      + h11 * dt * v1;
  }

  if (t >= 1.0f) {
    g_currentFrame++;
    if (g_currentFrame >= g_currentGesture->count) {
      g_gestureState = GESTURE_IDLE;
      Serial.println("[GESTURE] Complete");
      return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) g_frameStart[i] = g_jointTargets[i];
    g_frameStartMs = millis();
    applyFrameSpeed(g_currentGesture->frames[g_currentFrame]);
  }
}

void applyJointTargets() {
  if (g_sysState == SYS_WAITING || g_sysState == SYS_MONITORING) return;

  for (int i = 0; i < (int)leftArm.size(); i++) {
    float safe = constrain(g_jointTargets[i], JOINT_MIN[i], JOINT_MAX[i]);
    leftArm[i].setTargetPositionDeg(safe);
    leftArm[i].update();
  }

  for (int i = 0; i < (int)rightArm.size(); i++) {
    float safe = constrain(g_jointTargets[i + 6], JOINT_MIN[i + 6], JOINT_MAX[i + 6]);
    rightArm[i].setTargetPositionDeg(safe);
    rightArm[i].update();
  }

  float rmcsL = constrain(g_jointTargets[4], JOINT_MIN[4], JOINT_MAX[4]);
  if (fabsf(rmcsL - g_prevRmcsLeft) > 0.1f) {
    rmcsLeft.writeAngle(rmcsL);
    g_prevRmcsLeft = rmcsL;
  }

  float rmcsR = constrain(g_jointTargets[10], JOINT_MIN[10], JOINT_MAX[10]);
  if (fabsf(rmcsR - g_prevRmcsRight) > 0.1f) {
    rmcsRight.writeAngle(rmcsR);
    g_prevRmcsRight = rmcsR;
  }

  float servoL = constrain(g_jointTargets[5], JOINT_MIN[5], JOINT_MAX[5]) + 90.0f;
  if (fabsf(servoL - g_prevServoLeft) > 0.5f) {
    wristLeftServo.write(constrain((int)servoL, 0, 180));
    g_prevServoLeft = servoL;
  }

  float servoR = constrain(g_jointTargets[11], JOINT_MIN[11], JOINT_MAX[11]) + 90.0f;
  if (fabsf(servoR - g_prevServoRight) > 0.5f) {
    wristRightServo.write(constrain((int)servoR, 0, 180));
    g_prevServoRight = servoR;
  }
}

void printMotorStatus() {
  Serial.print("[MOTORS] CAN1:");
  for (auto &m : leftArm) {
    Serial.print(" 0x");
    if (m.getMotorId() < 0x10) Serial.print("0");
    Serial.print(m.getMotorId(), HEX);
    Serial.print(m.hasFeedback() ? "=ONLINE" : "=offline");
  }
  Serial.print("  |  CAN3:");
  for (auto &m : rightArm) {
    Serial.print(" 0x");
    if (m.getMotorId() < 0x10) Serial.print("0");
    Serial.print(m.getMotorId(), HEX);
    Serial.print(m.hasFeedback() ? "=ONLINE" : "=offline");
  }
  Serial.println();
}

void checkMotorReadiness() {
  if (g_sysState != SYS_WAITING) return;

  int readyCount  = 0;
  int expectCount = 0;
  for (auto &m : leftArm)  { expectCount++; if (m.hasFeedback()) readyCount++; }
  for (auto &m : rightArm) { expectCount++; if (m.hasFeedback()) readyCount++; }

  bool allReady = (readyCount == expectCount);
  bool timedOut = (millis() - g_waitStartMs) > MOTOR_WAIT_TIMEOUT_MS;
  if (!allReady && !timedOut) return;

  if (timedOut && !allReady) {
    Serial.print("[WARN] Motor wait timeout! Ready=");
    Serial.print(readyCount); Serial.print("/"); Serial.print(expectCount);
    Serial.print(". Missing IDs:");
    for (auto &m : leftArm)  if (!m.hasFeedback()) { Serial.print(" L0x"); Serial.print(m.getMotorId(), HEX); }
    for (auto &m : rightArm) if (!m.hasFeedback()) { Serial.print(" R0x"); Serial.print(m.getMotorId(), HEX); }
    Serial.println(" — proceeding anyway");
  } else {
    Serial.print("[SYS] All "); Serial.print(readyCount); Serial.println(" CubeMars motors confirmed online");
  }
  printMotorStatus();

  g_sysState = SYS_MONITORING;
  Serial.println("[SYS] ══ MONITORING ══ — showing raw motor positions (origin NOT set yet)");
  Serial.println("[SYS] NO motor commands will be sent.");
  Serial.println("[SYS] Position the arm as needed, then type any key + Enter to SET ORIGIN and enable motion.");
}

// ── Serial input buffering ────────────────────────────────────────────────────
static char    g_serialBuf[16];
static uint8_t g_serialIdx = 0;
static bool    g_serialLineReady = false;

void checkSerialReady() {
  static uint32_t lastPrintMs = 0;
  if (g_sysState != SYS_MONITORING) return;

  if (millis() - lastPrintMs >= 500) {
    lastPrintMs = millis();
    Serial.print("[ANGLES] R[ ");
    for (size_t i = 0; i < rightArm.size(); ++i) {
      Serial.print(rightArm[i].hasFeedback() ? rightArm[i].getCurPosDeg() : -999.0f, 2);
      if (i < rightArm.size() - 1) Serial.print("  ");
    }
    Serial.print(" ]  L[ ");
    for (size_t i = 0; i < leftArm.size(); ++i) {
      Serial.print(leftArm[i].hasFeedback() ? leftArm[i].getCurPosDeg() : -999.0f, 2);
      if (i < leftArm.size() - 1) Serial.print("  ");
    }
    Serial.print(" ]  ← ");
    if (!g_originSet) {
      Serial.println("press any key + Enter to SET ORIGIN here (no motion yet)");
    } else {
      Serial.println("press Enter to ENABLE MOTORS (will hold current pose)");
    }
  }

  if (!g_serialLineReady) return;
  g_serialLineReady = false;

  if (!g_originSet) {
    Serial.println("[SYS] Setting origins at CURRENT physical position (no motion) ...");
    for (auto &m : leftArm)  { if (m.hasFeedback()) { m.setOrigin(0); delay(5); } }
    for (auto &m : rightArm) { if (m.hasFeedback()) { m.setOrigin(0); delay(5); } }
    Serial.println("[SYS] Waiting 500 ms for origins to settle...");
    delay(500);
    rmcsLeft.resetEncoder();
    rmcsRight.resetEncoder();
    for (int i = 0; i < NUM_JOINTS; ++i) g_jointTargets[i] = 0.0f;

    Serial.print("[ANGLES AFTER ORIGIN] R[ ");
    for (size_t i = 0; i < rightArm.size(); ++i) {
      Serial.print(rightArm[i].hasFeedback() ? rightArm[i].getCurPosDeg() : -999.0f, 2);
      if (i < rightArm.size() - 1) Serial.print("  ");
    }
    Serial.print(" ]  L[ ");
    for (size_t i = 0; i < leftArm.size(); ++i) {
      Serial.print(leftArm[i].hasFeedback() ? leftArm[i].getCurPosDeg() : -999.0f, 2);
      if (i < leftArm.size() - 1) Serial.print("  ");
    }
    Serial.println(" ]");

    g_originSet = true;
    Serial.println("[SYS] Origin set. Still in MONITORING — press Enter again to ENABLE MOTORS.");
    return;
  }

  Serial.println("[SYS] Enabling motors at current pose (targets = 0 in new frame).");
  g_sysState = SYS_READY;
  Serial.println("[SYS] ══ READY ══ — motion enabled, waiting for Jetson commands.");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Head hardware
// ═══════════════════════════════════════════════════════════════════════════════

// ── PCA9685 channels ──────────────────────────────────────────────────────────
#define CH_EYE_TL         2
#define CH_EYE_TR         3
#define CH_EYE_BL         4
#define CH_EYE_BR         5
#define CH_JAW            14
#define CH_LIP            15

// ── Servo pulse constants ─────────────────────────────────────────────────────
#define SERVO_MIN         150
#define SERVO_MAX         600
#define PCA_FREQ          50

// ── Neck servos ───────────────────────────────────────────────────────────────
#define NECK_PIN          15
#define NECK_PIN_Y        14
#define NECK_MIN_DEG      0
#define NECK_MAX_DEG      180
#define NECK_CENTER_DEG   100.0f
#define NECK_LEFT_DEG     135.0f
#define NECK_RIGHT_DEG    45.0f

// ── Eye positions ─────────────────────────────────────────────────────────────
#define EYE_OPEN_TICK     ((SERVO_MIN + SERVO_MAX) / 2 + 80)
#define EYE_CLOSED_TICK   SERVO_MIN

// ── Jaw / Lip positions ───────────────────────────────────────────────────────
#define JAW_REST_TICK     SERVO_MIN
#define JAW_OPEN_TICK     (SERVO_MIN + 120)
#define LIP_REST_TICK     SERVO_MIN
#define LIP_OPEN_TICK     (SERVO_MIN + 80)

// ── Talk oscillator bounds ────────────────────────────────────────────────────
#define TALK_FREQ_MIN     1.2f
#define TALK_FREQ_MAX     2.8f
#define TALK_AMP_MIN      0.12f
#define TALK_AMP_MAX      0.50f
#define TALK_VAR_MIN_MS   280
#define TALK_VAR_MAX_MS   520
#define TALK_AMP_LERP     0.08f

// ── Easing ────────────────────────────────────────────────────────────────────
#define EASE_STEPS        40
#define EASE_STEP_MS      10
#define BLINK_CLOSE_MS    80
#define BLINK_OPEN_MS     60

// ── Head objects ──────────────────────────────────────────────────────────────
Servo                    neckServo;
Servo                    neckServoY;
Adafruit_PWMServoDriver  pca = Adafruit_PWMServoDriver(0x40, Wire2);

// ── Neck easing state ─────────────────────────────────────────────────────────
float    g_neckCurrent = NECK_CENTER_DEG;
float    g_neckTarget  = NECK_CENTER_DEG;
float    g_neckStart   = NECK_CENTER_DEG;
bool     g_neckMoving  = false;
int      g_easeStep    = 0;
uint32_t g_lastStepMs  = 0;

// ── Talk state ────────────────────────────────────────────────────────────────
bool     g_talking       = false;
float    g_talkPhase     = 0.0f;
float    g_talkFreq      = 2.0f;
float    g_talkAmp       = 0.3f;
float    g_talkAmpTarget = 0.3f;
uint32_t g_talkLastMs    = 0;
uint32_t g_talkNextVarMs = 0;

// ── Blink state ───────────────────────────────────────────────────────────────
enum BlinkState { BLINK_IDLE, BLINK_CLOSING, BLINK_OPENING };
BlinkState g_blinkState = BLINK_IDLE;
uint32_t   g_blinkMs    = 0;

// ── Head helpers ──────────────────────────────────────────────────────────────
void pcaWrite(uint8_t ch, int tick) {
    pca.setPWM(ch, 0, constrain(tick, SERVO_MIN, SERVO_MAX));
}

float easeOutQuart(float t) {
    float u = 1.0f - t;
    return 1.0f - u * u * u * u;
}

void setNeckImmediate(float deg) {
    deg = constrain(deg, (float)NECK_MIN_DEG, (float)NECK_MAX_DEG);
    neckServo.write((int)deg);
    g_neckCurrent = deg;
}

void startNeckMove(float targetDeg) {
    targetDeg = constrain(targetDeg, (float)NECK_MIN_DEG, (float)NECK_MAX_DEG);
    if (fabsf(targetDeg - g_neckTarget) < 0.5f && g_neckMoving) return;
    g_neckStart  = g_neckCurrent;
    g_neckTarget = targetDeg;
    g_easeStep   = 0;
    g_neckMoving = true;
    g_lastStepMs = millis();
}

void updateNeckEasing() {
    if (!g_neckMoving) return;
    uint32_t now = millis();
    if (now - g_lastStepMs < EASE_STEP_MS) return;
    g_lastStepMs = now;
    g_easeStep++;
    if (g_easeStep >= EASE_STEPS) {
        setNeckImmediate(g_neckTarget);
        g_neckMoving = false;
        return;
    }
    float t   = (float)g_easeStep / (float)EASE_STEPS;
    float deg = g_neckStart + (g_neckTarget - g_neckStart) * easeOutQuart(t);
    setNeckImmediate(deg);
}

void setEyes(int tick) {
    pcaWrite(CH_EYE_TL, tick);
    pcaWrite(CH_EYE_TR, tick);
    pcaWrite(CH_EYE_BL, tick);
    pcaWrite(CH_EYE_BR, tick);
}

void setJawLip(bool open) {
    pcaWrite(CH_JAW, open ? JAW_OPEN_TICK : JAW_REST_TICK);
    pcaWrite(CH_LIP, open ? LIP_OPEN_TICK : LIP_REST_TICK);
}

void setJawLipF(float openness) {
    openness = constrain(openness, 0.0f, 1.0f);
    pcaWrite(CH_JAW, JAW_REST_TICK + (int)(openness * (JAW_OPEN_TICK - JAW_REST_TICK)));
    pcaWrite(CH_LIP, LIP_REST_TICK + (int)(openness * (LIP_OPEN_TICK - LIP_REST_TICK)));
}

void updateTalkAnimation() {
    if (!g_talking) return;
    uint32_t now = millis();
    float dt = (now - g_talkLastMs) * 0.001f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.016f;
    g_talkLastMs = now;

    if (now >= g_talkNextVarMs) {
        g_talkFreq      = TALK_FREQ_MIN
                        + random(0, (int)((TALK_FREQ_MAX - TALK_FREQ_MIN) * 10)) * 0.1f;
        g_talkAmpTarget = TALK_AMP_MIN
                        + random(0, (int)((TALK_AMP_MAX - TALK_AMP_MIN) * 100)) * 0.01f;
        g_talkNextVarMs = now + TALK_VAR_MIN_MS + random(0, TALK_VAR_MAX_MS - TALK_VAR_MIN_MS);
    }
    g_talkAmp += (g_talkAmpTarget - g_talkAmp) * TALK_AMP_LERP;
    g_talkPhase += TWO_PI * g_talkFreq * dt;
    if (g_talkPhase >= TWO_PI) g_talkPhase -= TWO_PI;
    setJawLipF(fabsf(sinf(g_talkPhase)) * g_talkAmp);
}

void updateBlink() {
    if (g_blinkState == BLINK_IDLE) return;
    uint32_t now = millis();
    if (g_blinkState == BLINK_CLOSING && now - g_blinkMs >= BLINK_CLOSE_MS) {
        g_blinkState = BLINK_OPENING;
        g_blinkMs    = now;
        setEyes(EYE_OPEN_TICK);
    } else if (g_blinkState == BLINK_OPENING && now - g_blinkMs >= BLINK_OPEN_MS) {
        g_blinkState = BLINK_IDLE;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  micro-ROS
// ═══════════════════════════════════════════════════════════════════════════════

rcl_node_t          node;
rcl_subscription_t  arm_state_sub;
rcl_subscription_t  joint_targets_sub;
rcl_subscription_t  talk_sub;
rcl_subscription_t  neck_angle_sub;
rclc_executor_t     executor;
rclc_support_t      support;
rcl_allocator_t     allocator;

// arm_state message
#define CMD_BUF_MAX 32
char                     cmd_buf[CMD_BUF_MAX];
std_msgs__msg__String    arm_state_msg;

// talk_state message
char                     head_cmd_buf[CMD_BUF_MAX];
std_msgs__msg__String    talk_msg;

// joint_targets message
float                                joint_data[NUM_JOINTS];
std_msgs__msg__Float32MultiArray     joint_targets_msg;

// neck_angle message
std_msgs__msg__UInt16  angle_msg;

// ── arm_state callback ────────────────────────────────────────────────────────
void arm_state_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  if (!msg->data.data || msg->data.size == 0) return;
  const char *cmd = msg->data.data;

  if (g_sysState != SYS_READY) {
    const char* stateStr = (g_sysState == SYS_WAITING)   ? "WAITING" :
                           (g_sysState == SYS_MONITORING) ? "MONITORING" : "HOMING";
    Serial.print("[CMD] Ignored (system not READY, state=");
    Serial.print(stateStr);
    Serial.println(")");
    return;
  }

  Serial.print("[CMD] arm_state: \"");
  Serial.print(cmd);
  Serial.println("\"");

  if      (strncmp(cmd, "hello",     5) == 0) startGesture(&GESTURE_HELLO);
  else if (strncmp(cmd, "bye",       3) == 0) startGesture(&GESTURE_BYE);
  else if (strncmp(cmd, "home",      4) == 0) startGesture(&GESTURE_HOME);
  else if (strncmp(cmd, "namaste",   7) == 0) startGesture(&GESTURE_NAMASTE);
  else if (strncmp(cmd, "handshake", 9) == 0) startGesture(&GESTURE_HANDSHAKE);
  else if (strncmp(cmd, "stop",      4) == 0) {
    g_gestureState = GESTURE_IDLE;
    Serial.println("[CMD] STOP — gesture aborted, holding position");
  } else {
    Serial.print("[CMD] Unknown arm command: "); Serial.println(cmd);
  }
}

// ── joint_targets callback ────────────────────────────────────────────────────
void joint_targets_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  if (g_sysState != SYS_READY) {
    Serial.println("[TUNING] Ignored — system not READY");
    return;
  }
  if (g_gestureState == GESTURE_PLAYING) {
    g_gestureState = GESTURE_IDLE;
    Serial.println("[TUNING] Gesture interrupted by joint_targets");
  }
  int count = (msg->data.size < NUM_JOINTS) ? msg->data.size : NUM_JOINTS;
  for (int i = 0; i < count; i++)
    g_jointTargets[i] = constrain(msg->data.data[i], JOINT_MIN[i], JOINT_MAX[i]);
}

// ── talk_state callback ───────────────────────────────────────────────────────
void talk_state_callback(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    if (!msg->data.data || msg->data.size == 0) return;
    const char *cmd = msg->data.data;

    if (strncmp(cmd, "talking", 7) == 0) {
        if (!g_talking) {
            g_talkPhase     = 0.0f;
            g_talkFreq      = 2.0f;
            g_talkAmp       = 0.3f;
            g_talkAmpTarget = 0.3f;
            g_talkLastMs    = millis();
            g_talkNextVarMs = millis();
        }
        g_talking = true;
    } else if (strncmp(cmd, "not_talking", 11) == 0) {
        g_talking = false;
        setJawLip(false);
    } else if (strncmp(cmd, "blink", 5) == 0) {
        if (g_blinkState == BLINK_IDLE) {
            g_blinkState = BLINK_CLOSING;
            g_blinkMs    = millis();
            setEyes(EYE_CLOSED_TICK);
        }
    } else if (strncmp(cmd, "left",  4) == 0) {
        startNeckMove(NECK_LEFT_DEG);
    } else if (strncmp(cmd, "right", 5) == 0) {
        startNeckMove(NECK_RIGHT_DEG);
    } else if (strncmp(cmd, "front", 5) == 0) {
        startNeckMove(NECK_CENTER_DEG);
    } else {
        Serial.print("[CMD] Unknown talk command: "); Serial.println(cmd);
    }
}

// ── neck_angle callback ───────────────────────────────────────────────────────
void neck_angle_callback(const void *msgin) {
    const std_msgs__msg__UInt16 *msg = (const std_msgs__msg__UInt16 *)msgin;
    Serial.print("[NECK] received angle="); Serial.println(msg->data);
    startNeckMove((float)msg->data);
}

// ── Error loop ────────────────────────────────────────────────────────────────
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
  Serial.print("[RCCHECK FAIL] "); Serial.print(#fn); \
  Serial.print(" rc="); Serial.println(rc); errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
  Serial.print("[RCSOFTCHECK] "); Serial.println(#fn); } }

void errorLoop() {
  Serial.println("[ERROR] micro-ROS init failed — halting (LED blinks rapidly)");
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);

  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║   EVA FULL BODY — BOOT                       ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println("Firmware: eva_full_body (arms + head animatronics)");

  // ── Ethernet micro-ROS transport ──
  Serial.println("[SETUP] Initializing micro-ROS Ethernet transport...");
  set_microros_transports();
  delay(2000);

  // ── CAN buses — init only, no motor commands yet ──
  Serial.println("[SETUP] CAN1 (left arm)...");
  leftBus.begin(1000000);
  leftBus.attachMotors(leftArm);

  Serial.println("[SETUP] CAN3 (right arm)...");
  rightBus.begin(1000000);
  rightBus.attachMotors(rightArm);

  // ── RMCS I2C init ──
  Serial.println("[SETUP] RMCS I2C bus init...");
  rmcsLeft.begin();
  rmcsRight.begin();

  // ── Wrist servos — center immediately ──
  Serial.println("[SETUP] Wrist servos → center (90°)...");
  wristLeftServo.attach(SERVO_PIN_LEFT);
  wristRightServo.attach(SERVO_PIN_RIGHT);
  wristLeftServo.write(90);
  wristRightServo.write(90);

  // ── Head hardware ──
  Serial.println("[SETUP] Neck servos...");
  neckServo.attach(NECK_PIN);
  neckServoY.attach(NECK_PIN_Y);
  neckServoY.write(90);
  setNeckImmediate(NECK_CENTER_DEG);

  Serial.println("[SETUP] PCA9685 (Wire2, 0x40)...");
  Wire2.begin();
  pca.begin();
  pca.setPWMFreq(PCA_FREQ);
  setEyes(EYE_OPEN_TICK);
  setJawLip(false);

  // ── micro-ROS messages — pre-allocate buffers ──
  arm_state_msg.data.data     = cmd_buf;
  arm_state_msg.data.size     = 0;
  arm_state_msg.data.capacity = CMD_BUF_MAX;

  talk_msg.data.data     = head_cmd_buf;
  talk_msg.data.size     = 0;
  talk_msg.data.capacity = CMD_BUF_MAX;

  joint_targets_msg.data.data     = joint_data;
  joint_targets_msg.data.size     = 0;
  joint_targets_msg.data.capacity = NUM_JOINTS;

  // ── micro-ROS node + subscriptions ──
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "eva_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &arm_state_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "arm_state"
  ));
  RCCHECK(rclc_subscription_init_default(
    &joint_targets_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "joint_targets"
  ));
  RCCHECK(rclc_subscription_init_default(
    &talk_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "talk_state"
  ));
  RCCHECK(rclc_subscription_init_default(
    &neck_angle_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "neck_angle"
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &arm_state_sub, &arm_state_msg, &arm_state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &joint_targets_sub, &joint_targets_msg, &joint_targets_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &talk_sub, &talk_msg, &talk_state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &neck_angle_sub, &angle_msg, &neck_angle_callback, ON_NEW_DATA));

  g_waitStartMs = millis();
  g_sysState    = SYS_WAITING;

  Serial.println("[SETUP] micro-ROS ready");
  Serial.println("  Node:   eva_node");
  Serial.println("  Topics: /arm_state, /joint_targets, /talk_state, /neck_angle");
  Serial.println("[SYS] ══ WAITING for arm motors ══ (head animatronics active immediately)");
  Serial.println("  Power motor supply now if not already on.");
  Serial.println("══════════════════════════════════════════════════\n");
}

// ── Serial reset + gesture command handler ────────────────────────────────────
void checkSerialReset() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      g_serialBuf[g_serialIdx] = '\0';
      if (g_serialIdx > 0 && strcmp(g_serialBuf, "reset") == 0) {
        Serial.println("\n[SYS] ══ RESET ══ — rebooting Teensy...");
        Serial.flush();
        _reboot_Teensyduino_();
      } else if (g_serialIdx > 0) {
        // Head commands work regardless of SYS_READY (head is never gated)
        if (strcmp(g_serialBuf, "blink") == 0) {
          if (g_blinkState == BLINK_IDLE) {
            g_blinkState = BLINK_CLOSING;
            g_blinkMs    = millis();
            setEyes(EYE_CLOSED_TICK);
            Serial.println("[HEAD] blink");
          }
        } else if (strcmp(g_serialBuf, "talking") == 0) {
          if (!g_talking) {
            g_talkPhase     = 0.0f;
            g_talkFreq      = 2.0f;
            g_talkAmp       = 0.3f;
            g_talkAmpTarget = 0.3f;
            g_talkLastMs    = millis();
            g_talkNextVarMs = millis();
          }
          g_talking = true;
          Serial.println("[HEAD] talking ON");
        } else if (strcmp(g_serialBuf, "not_talking") == 0) {
          g_talking = false;
          setJawLip(false);
          Serial.println("[HEAD] talking OFF");
        } else if (strcmp(g_serialBuf, "left") == 0) {
          startNeckMove(NECK_LEFT_DEG);
          Serial.println("[HEAD] neck left");
        } else if (strcmp(g_serialBuf, "right") == 0) {
          startNeckMove(NECK_RIGHT_DEG);
          Serial.println("[HEAD] neck right");
        } else if (strcmp(g_serialBuf, "front") == 0) {
          startNeckMove(NECK_CENTER_DEG);
          Serial.println("[HEAD] neck front");
        } else if (g_sysState == SYS_READY) {
          if      (strcmp(g_serialBuf, "hello")     == 0) startGesture(&GESTURE_HELLO);
          else if (strcmp(g_serialBuf, "bye")       == 0) startGesture(&GESTURE_BYE);
          else if (strcmp(g_serialBuf, "home")      == 0) startGesture(&GESTURE_HOME);
          else if (strcmp(g_serialBuf, "namaste")   == 0) startGesture(&GESTURE_NAMASTE);
          else if (strcmp(g_serialBuf, "handshake") == 0) startGesture(&GESTURE_HANDSHAKE);
          else if (strcmp(g_serialBuf, "stop")      == 0) {
            g_gestureState = GESTURE_IDLE;
            Serial.println("[CMD] STOP — gesture aborted, holding position");
          } else if (strcmp(g_serialBuf, "?") == 0 || strcmp(g_serialBuf, "help") == 0) {
            Serial.println("[SERIAL] Head:  blink | talking | not_talking | left | right | front");
            Serial.println("[SERIAL] Arms:  hello | bye | home | namaste | handshake | stop | reset");
          } else {
            Serial.print("[SERIAL] Unknown: '");
            Serial.print(g_serialBuf);
            Serial.println("'  (type 'help' for list)");
          }
        } else if (strcmp(g_serialBuf, "?") == 0 || strcmp(g_serialBuf, "help") == 0) {
          Serial.println("[SERIAL] Head:  blink | talking | not_talking | left | right | front  (arms not ready yet)");
        } else {
          g_serialLineReady = true;
        }
      } else {
        g_serialLineReady = true;
      }
      g_serialIdx = 0;
    } else if (g_serialIdx < sizeof(g_serialBuf) - 1) {
      g_serialBuf[g_serialIdx++] = c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════════════════════

static uint32_t lastStatusMs  = 0;
static uint32_t lastWaitLogMs = 0;

void loop() {
  // 1. Spin micro-ROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // 2. Serial input
  checkSerialReset();

  // 3. Arm safety state machine
  checkMotorReadiness();
  checkSerialReady();

  // 4. Arm gesture + output
  updateGesture();
  applyJointTargets();

  // 5. Head animatronics (always active)
  updateNeckEasing();
  updateTalkAnimation();
  updateBlink();

  // 6. Periodic waiting log (every 2s)
  if (g_sysState == SYS_WAITING && (millis() - lastWaitLogMs >= 2000)) {
    lastWaitLogMs = millis();
    int ready = 0, expect = 0;
    for (auto &m : leftArm)  { expect++; if (m.hasFeedback()) ready++; }
    for (auto &m : rightArm) { expect++; if (m.hasFeedback()) ready++; }
    uint32_t remaining = MOTOR_WAIT_TIMEOUT_MS - (millis() - g_waitStartMs);
    Serial.print("[SYS] Waiting for motors... ");
    Serial.print(ready); Serial.print("/"); Serial.print(expect);
    Serial.print(" online, timeout in "); Serial.print(remaining / 1000); Serial.println("s");
    printMotorStatus();
  }

  // 7. Status print (1 Hz, only when READY)
  if (g_sysState == SYS_READY && (millis() - lastStatusMs >= 1000)) {
    lastStatusMs = millis();
    Serial.print("[STATUS] ");
    Serial.print(g_gestureState == GESTURE_IDLE ? "IDLE" : "PLAYING");
    Serial.print(" | joints=[");
    for (int i = 0; i < NUM_JOINTS; i++) {
      Serial.print(g_jointTargets[i], 1);
      if (i < NUM_JOINTS - 1) Serial.print(", ");
    }
    Serial.println("]");
  }

  delay(10);  // ~100 Hz control loop
}
