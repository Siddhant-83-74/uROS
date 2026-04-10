#include "CubeMarsServoDual.h"
#include <Wire.h>
#include "RMCS-220X_Right.h"
#include "RMCS-220X_Left.h"
#include <Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

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

// ── Hardware config ─────────────────────────────────────────────────────────
#define RMCS_ADDR       0x08
#define SERVO_PIN_LEFT  10
#define SERVO_PIN_RIGHT 1

// ── CAN bus instances ───────────────────────────────────────────────────────
CubeMarsCAN<CAN1> leftBus;
CubeMarsCAN<CAN3> rightBus;

std::vector<CubeMarsMotor> leftArm = {
  // CubeMarsMotor(0x01, leftBus, 6, 4000, 6000),
  // CubeMarsMotor(0x02, leftBus, 6, 4000, 6000),
  // CubeMarsMotor(0x03, leftBus, 6, 4000, 6000),
  // CubeMarsMotor(0x04, leftBus, 6, 4000, 6000),
};

// TEMPORARY: single motor (0x05) for origin testing
std::vector<CubeMarsMotor> rightArm = {
  CubeMarsMotor(0x05, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x06, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x07, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x08, rightBus, 6, 4000, 6000),
};
bool g_originSet = false;   // false until we run setOrigin() once

// ── RMCS motors ─────────────────────────────────────────────────────────────
RMCS220X_Left  rmcsLeft;
RMCS220X_Right rmcsRight;

// ── Wrist servos ────────────────────────────────────────────────────────────
Servo wristLeftServo;
Servo wristRightServo;

// ═══════════════════════════════════════════════════════════════════════════════
//  System State Machine
//
//  SYS_WAITING    → no CAN commands sent, polling for motor feedback
//  SYS_MONITORING → origins set, printing live angles, ZERO commands sent,
//                   waiting for operator to type anything in Serial monitor
//  SYS_READY      → full operation: gesture commands and tuning accepted
//
//  This prevents the dangerous power-sequencing jerk where the Teensy boots
//  before motors, causing motors to receive "go to 0°" the instant they power
//  on — potentially from any physical position.
// ═══════════════════════════════════════════════════════════════════════════════

enum SysState { SYS_WAITING, SYS_HOMING, SYS_MONITORING, SYS_READY };
SysState g_sysState = SYS_WAITING;

#define MOTOR_WAIT_TIMEOUT_MS 1000000   // give up waiting after 1000s
uint32_t g_waitStartMs = 0;

// ── Soft position limits (degrees) ──────────────────────────────────────────
// Conservative defaults — tighten these after physical testing.
// Joints:                  0      1      2      3      4      5
const float JOINT_MIN[NUM_JOINTS] = { -90,  -45,  -90,  -90,  -90,  -80,
//                         6      7      8      9     10     11
                            -90,  -45,  -90,  -90,  -90,  -80 };
const float JOINT_MAX[NUM_JOINTS] = {  90,   45,   90,   90,   90,   80,
                                        90,   45,   90,   90,   90,   80 };

// ═══════════════════════════════════════════════════════════════════════════════
//  Gesture engine
// ═══════════════════════════════════════════════════════════════════════════════

struct Keyframe {
  float    joints[NUM_JOINTS];
  uint32_t duration_ms;          // time to interpolate from previous frame to this
};

#define MAX_KEYFRAMES 10

struct Gesture {
  Keyframe frames[MAX_KEYFRAMES];
  uint8_t  count;
};

// Current joint targets (written by gesture engine or tuning subscriber)
float g_jointTargets[NUM_JOINTS] = {0};

// Previous RMCS/servo targets — only send I2C/PWM when changed
float g_prevRmcsLeft   = -999.0f;   // force first write on enable
float g_prevRmcsRight  = -999.0f;
float g_prevServoLeft  = -999.0f;
float g_prevServoRight = -999.0f;

// ── Gesture state machine ───────────────────────────────────────────────────
enum GestureState { GESTURE_IDLE, GESTURE_PLAYING };
GestureState g_gestureState = GESTURE_IDLE;

const Gesture *g_currentGesture = nullptr;
uint8_t  g_currentFrame   = 0;
float    g_frameStart[NUM_JOINTS] = {0};
uint32_t g_frameStartMs   = 0;

// ── Predefined gestures ─────────────────────────────────────────────────────
// NOTE: All gestures end at {0,...,0} (home/rest position).
// PLACEHOLDER ANGLES — tune with arm_tuner.py on the Jetson.
// Start conservative (small angles) and increase after verifying safe range.

// Startup-only: slow move from wherever motors are to zero. 3000ms gives
// CubeMars motors (4000 ERPM speed) time to move safely from any position.
const Gesture GESTURE_HOME_SLOW = {
  .frames = {
    { .joints = {0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0}, .duration_ms = 3000 },
  },
  .count = 1,
};

const Gesture GESTURE_HOME = {
  .frames = {
    { .joints = {0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0}, .duration_ms = 1000 },
  },
  .count = 1,
};

const Gesture GESTURE_HELLO = {
  .frames = {
    // 1. Raise right arm
    { .joints = {0, 0, 0, 0, 0, 0,   -30, 20, 0, -45, 0, 0}, .duration_ms = 800 },
    // 2. Wave right (wrist + forearm)
    { .joints = {0, 0, 0, 0, 0, 0,   -30, 20, 0, -45, 15, 20}, .duration_ms = 400 },
    // 3. Wave left
    { .joints = {0, 0, 0, 0, 0, 0,   -30, 20, 0, -45, -15, -20}, .duration_ms = 400 },
    // 4. Wave right again
    { .joints = {0, 0, 0, 0, 0, 0,   -30, 20, 0, -45, 15, 20}, .duration_ms = 400 },
    // 5. Return to rest
    { .joints = {0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0}, .duration_ms = 800 },
  },
  .count = 5,
};

const Gesture GESTURE_BYE = {
  .frames = {
    // 1. Raise both arms slightly
    { .joints = {-20, 15, 0, -30, 0, 0,   -20, 15, 0, -30, 0, 0}, .duration_ms = 800 },
    // 2. Wave out
    { .joints = {-20, 25, 0, -30, 10, 15,   -20, 25, 0, -30, 10, 15}, .duration_ms = 400 },
    // 3. Wave in
    { .joints = {-20, 15, 0, -30, -10, -15,   -20, 15, 0, -30, -10, -15}, .duration_ms = 400 },
    // 4. Wave out again
    { .joints = {-20, 25, 0, -30, 10, 15,   -20, 25, 0, -30, 10, 15}, .duration_ms = 400 },
    // 5. Return to rest
    { .joints = {0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0}, .duration_ms = 800 },
  },
  .count = 5,
};

// ── Start a gesture ─────────────────────────────────────────────────────────
void startGesture(const Gesture *gesture) {
  g_currentGesture = gesture;
  g_currentFrame   = 0;
  g_gestureState   = GESTURE_PLAYING;
  g_frameStartMs   = millis();

  // Snapshot current targets as the starting point for interpolation
  for (int i = 0; i < NUM_JOINTS; i++) {
    g_frameStart[i] = g_jointTargets[i];
  }

  Serial.print("[GESTURE] Started, frames=");
  Serial.println(gesture->count);
}

// ── Non-blocking gesture update ─────────────────────────────────────────────
void updateGesture() {
  if (g_gestureState != GESTURE_PLAYING || !g_currentGesture) return;

  const Keyframe &target = g_currentGesture->frames[g_currentFrame];
  uint32_t elapsed = millis() - g_frameStartMs;

  float t = (target.duration_ms > 0)
            ? (float)elapsed / (float)target.duration_ms
            : 1.0f;

  if (t > 1.0f) t = 1.0f;

  // Ease-out-quart: fast start, smooth deceleration
  float u = 1.0f - t;
  float e = 1.0f - u * u * u * u;

  // Interpolate all joints
  for (int i = 0; i < NUM_JOINTS; i++) {
    g_jointTargets[i] = g_frameStart[i] + (target.joints[i] - g_frameStart[i]) * e;
  }

  // Advance to next keyframe when this one completes
  if (t >= 1.0f) {
    g_currentFrame++;
    if (g_currentFrame >= g_currentGesture->count) {
      // Gesture finished
      g_gestureState = GESTURE_IDLE;

      Serial.println("[GESTURE] Complete");
      return;
    }
    // Snapshot for next interpolation segment
    for (int i = 0; i < NUM_JOINTS; i++) {
      g_frameStart[i] = g_jointTargets[i];
    }
    g_frameStartMs = millis();
  }
}

// ── Apply joint targets to hardware ─────────────────────────────────────────
// SAFETY: Does nothing while system is in SYS_WAITING state.
// All targets are clamped to JOINT_MIN/JOINT_MAX before being sent.
void applyJointTargets() {
  // Block all hardware commands until operator explicitly enables motion via Serial
  if (g_sysState == SYS_WAITING || g_sysState == SYS_MONITORING) return;

  // CubeMars left arm (joints 0–3)
  // for (int i = 0; i < 4; i++) {
  //   float safe = constrain(g_jointTargets[i], JOINT_MIN[i], JOINT_MAX[i]);
  //   leftArm[i].setTargetPositionDeg(safe);
  //   leftArm[i].update();
  // }

  // CubeMars right arm — TEMPORARY: only motor 0x05 (joint 6)
  for (int i = 0; i < rightArm.size(); i++) {
    float safe = constrain(g_jointTargets[i], JOINT_MIN[i], JOINT_MAX[i]);
    rightArm[i].setTargetPositionDeg(safe);
    rightArm[i].update();
  }

  // RMCS left forearm (joint 4) — only on change to avoid I2C spam
  float rmcsL = constrain(g_jointTargets[4], JOINT_MIN[4], JOINT_MAX[4]);
  if (fabsf(rmcsL - g_prevRmcsLeft) > 0.1f) {
    rmcsLeft.goToPositionInDegrees(rmcsL);
    g_prevRmcsLeft = rmcsL;
  }

  // RMCS right forearm (joint 10) — only on change
  float rmcsR = constrain(g_jointTargets[10], JOINT_MIN[10], JOINT_MAX[10]);
  if (fabsf(rmcsR - g_prevRmcsRight) > 0.1f) {
    rmcsRight.goToPositionInDegrees(rmcsR);
    g_prevRmcsRight = rmcsR;
  }

  // Left wrist servo (joint 5) — offset by 90° (servo center = joint 0°)
  float servoL = constrain(g_jointTargets[5], JOINT_MIN[5], JOINT_MAX[5]) + 90.0f;
  if (fabsf(servoL - g_prevServoLeft) > 0.5f) {
    wristLeftServo.write(constrain((int)servoL, 0, 180));
    g_prevServoLeft = servoL;
  }

  // Right wrist servo (joint 11) — offset by 90°
  float servoR = constrain(g_jointTargets[11], JOINT_MIN[11], JOINT_MAX[11]) + 90.0f;
  if (fabsf(servoR - g_prevServoRight) > 0.5f) {
    wristRightServo.write(constrain((int)servoR, 0, 180));
    g_prevServoRight = servoR;
  }
}

// ── Motor readiness check ────────────────────────────────────────────────────
// Called every loop tick while in SYS_WAITING.
// Polls all 8 CubeMars motors for CAN feedback. Once all have responded
// (or timeout elapses), transitions to SYS_HOMING:
//   1. setOrigin(0) — define current physical position as zero
//   2. Initialize g_jointTargets from actual motor positions (no jerk)
//   3. Start slow homing gesture (3 seconds to zero)
void checkMotorReadiness() {
  if (g_sysState != SYS_WAITING) return;

  // Count how many motors have sent at least one feedback packet
  int readyCount  = 0;
  int expectCount = 0;
  // for (auto &m : leftArm)  { expectCount++; if (m.hasFeedback()) readyCount++; }
  for (auto &m : rightArm) { expectCount++; if (m.hasFeedback()) readyCount++; }

  bool allReady = (readyCount == expectCount);
  bool timedOut = (millis() - g_waitStartMs) > MOTOR_WAIT_TIMEOUT_MS;
  if (!allReady && !timedOut) return;

  // ── Log which motors (if any) are still missing ──
  if (timedOut && !allReady) {
    Serial.print("[WARN] Motor wait timeout! Ready=");
    Serial.print(readyCount); Serial.print("/"); Serial.print(expectCount);
    Serial.print(". Missing IDs:");
    // for (auto &m : leftArm)  if (!m.hasFeedback()) { Serial.print(" L0x"); Serial.print(m.getMotorId(), HEX); }
    for (auto &m : rightArm) if (!m.hasFeedback()) { Serial.print(" R0x"); Serial.print(m.getMotorId(), HEX); }
    Serial.println(" — proceeding anyway");
  } else {
    Serial.print("[SYS] All "); Serial.print(readyCount); Serial.println(" CubeMars motors confirmed online");
  }

  // ── Transition to MONITORING — show real physical positions BEFORE setOrigin ──
  // The operator sees where the arm actually is right now (in the motor's raw
  // encoder frame). Once they confirm and press Enter, THEN we call setOrigin(0)
  // to lock that physical position as 0°.
  // (Printing angles AFTER setOrigin is pointless — they'd always show 0.)
  g_sysState = SYS_MONITORING;
  Serial.println("[SYS] ══ MONITORING ══ — showing raw motor positions (origin NOT set yet)");
  Serial.println("[SYS] NO motor commands will be sent.");
  Serial.println("[SYS] Position the arm as needed, then type any key + Enter to SET ORIGIN and enable motion.");
}

// ── Serial input buffering (used by checkSerialReset + checkSerialReady) ────
static char g_serialBuf[16];
static uint8_t g_serialIdx = 0;
static bool g_serialLineReady = false;  // true when a non-reset line was entered

// Two-step safety gate for motion:
//   1) First key press in SYS_MONITORING  -> setOrigin() only, stay in MONITORING
//   2) Second key press                   -> switch to SYS_READY, start sending targets
//
// Motors will not receive *any* position commands until the second press,
// and the first target after enabling is 0° (which now equals the current pose).

void checkSerialReady() {
  static uint32_t lastPrintMs = 0;

  // Only active in monitoring state
  if (g_sysState != SYS_MONITORING) {
    return;
  }

  // 1) Periodic live angle printout
  if (millis() - lastPrintMs >= 500) {
    lastPrintMs = millis();

    Serial.print("[ANGLES] R[ ");
    for (size_t i = 0; i < rightArm.size(); ++i) {
      if (rightArm[i].hasFeedback()) {
        Serial.print(rightArm[i].getCurPosDeg(), 2);
      } else {
        Serial.print("---.--");
      }
      if (i < rightArm.size() - 1) Serial.print("  ");
    }

    Serial.print(" ]  L[ ");
    for (size_t i = 0; i < leftArm.size(); ++i) {
      if (leftArm[i].hasFeedback()) {
        Serial.print(leftArm[i].getCurPosDeg(), 2);
      } else {
        Serial.print("---.--");
      }
      if (i < leftArm.size() - 1) Serial.print("  ");
    }

    Serial.print(" ]  \u2190 ");

    if (!g_originSet) {
      Serial.println("press any key + Enter to SET ORIGIN here (no motion yet)");
    } else {
      Serial.println("press Enter to ENABLE MOTORS (will hold current pose)");
    }
  }

  // 2) No line entered -> nothing to do
  if (!g_serialLineReady) {
    return;
  }
  g_serialLineReady = false;

  // FIRST PRESS: set origin only, stay in SYS_MONITORING
  if (!g_originSet) {
    Serial.println("SYS Setting origins at CURRENT physical position (no motion) ...");

    // CubeMars left arm
    for (auto &m : leftArm) {
      if (m.hasFeedback()) {
        m.setOrigin(0);
        delay(5);
      }
    }

    // CubeMars right arm
    for (auto &m : rightArm) {
      if (m.hasFeedback()) {
        m.setOrigin(0);
        delay(5);
      }
    }

    // Let drivers process origin commands
    Serial.println("SYS Waiting 500 ms for origins to settle...");
    delay(500);

    // RMCS motors: define current pose as 0°
    rmcsLeft.calibrateEncoderPositionInDegrees(0);
    rmcsRight.calibrateEncoderPositionInDegrees(0);

    // After origin is set, zero all joint targets in this new frame.
    // 0° now means "stay exactly where you are right now".
    for (int i = 0; i < NUM_JOINTS; ++i) {
      g_jointTargets[i] = 0.0f;
    }
    // ONE-SHOT: print angles in the NEW frame (should be ~0)
    Serial.print("[ANGLES AFTER ORIGIN] R[ ");
    for (size_t i = 0; i < rightArm.size(); ++i) {
      if (rightArm[i].hasFeedback()) {
        Serial.print(rightArm[i].getCurPosDeg(), 2);
      } else {
        Serial.print("---.--");
      }
      if (i < rightArm.size() - 1) Serial.print("  ");
    }

    Serial.print(" ]  L[ ");
    for (size_t i = 0; i < leftArm.size(); ++i) {
      if (leftArm[i].hasFeedback()) {
        Serial.print(leftArm[i].getCurPosDeg(), 2);
      } else {
        Serial.print("---.--");
      }
      if (i < leftArm.size() - 1) Serial.print("  ");
    }
    Serial.println(" ]");

    g_originSet = true;
    Serial.println("SYS Origin set. Angles now are ~0 in this pose.");
    Serial.println("SYS Still in MONITORING (no motion). Press Enter again to ENABLE MOTORS.");
    return;  // remain in SYS_MONITORING, still no applyJointTargets()
  }

  // SECOND PRESS: enable motors, start sending targets
  Serial.println("SYS Enabling motors at current pose (targets = 0 in new frame).");
  g_sysState = SYS_READY;

  // From now on, loop() will call applyJointTargets(), which
  // sends 0° (i.e., hold here) until Jetson changes g_jointTargets[].

  Serial.println("SYS READY: motion enabled, waiting for Jetson commands.");
}


// ═══════════════════════════════════════════════════════════════════════════════
//  micro-ROS
// ═══════════════════════════════════════════════════════════════════════════════

rcl_node_t          node;
rcl_subscription_t  arm_state_sub;
rcl_subscription_t  joint_targets_sub;
rclc_executor_t     executor;
rclc_support_t      support;
rcl_allocator_t     allocator;

// arm_state message (String)
#define CMD_BUF_MAX 32
char                     cmd_buf[CMD_BUF_MAX];
std_msgs__msg__String    arm_state_msg;

// joint_targets message (Float32MultiArray)
float                                joint_data[NUM_JOINTS];
std_msgs__msg__Float32MultiArray     joint_targets_msg;

// ── arm_state callback ──────────────────────────────────────────────────────
void arm_state_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  if (!msg->data.data || msg->data.size == 0) return;
  const char *cmd = msg->data.data;

  // Reject commands until operator has confirmed motion is safe via Serial monitor
  if (g_sysState != SYS_READY) {
    const char* stateStr = (g_sysState == SYS_WAITING)    ? "WAITING"    :
                           (g_sysState == SYS_MONITORING)  ? "MONITORING" : "HOMING";
    Serial.print("[CMD] Ignored (system not READY, state=");
    Serial.print(stateStr);
    Serial.println(") — type anything in Serial monitor to enable");
    return;
  }

  Serial.print("[CMD] arm_state: \"");
  Serial.print(cmd);
  Serial.println("\"");

  if (strncmp(cmd, "hello", 5) == 0) {
    startGesture(&GESTURE_HELLO);
  } else if (strncmp(cmd, "bye", 3) == 0) {
    startGesture(&GESTURE_BYE);
  } else if (strncmp(cmd, "home", 4) == 0) {
    startGesture(&GESTURE_HOME);
  } else if (strncmp(cmd, "stop", 4) == 0) {
    // Emergency stop — abort gesture, stay at current positions
    g_gestureState = GESTURE_IDLE;
    Serial.println("[CMD] STOP — gesture aborted, holding position");
  } else {
    Serial.print("[CMD] Unknown command: ");
    Serial.println(cmd);
  }
}

// ── joint_targets callback (tuning mode) ────────────────────────────────────
void joint_targets_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  // Reject during startup — tuning is only safe when motors are ready
  if (g_sysState != SYS_READY) {
    Serial.println("[TUNING] Ignored — system not READY");
    return;
  }

  // Tuning input overrides gesture playback
  if (g_gestureState == GESTURE_PLAYING) {
    g_gestureState = GESTURE_IDLE;
    Serial.println("[TUNING] Gesture interrupted by joint_targets");
  }

  int count = (msg->data.size < NUM_JOINTS) ? msg->data.size : NUM_JOINTS;
  for (int i = 0; i < count; i++) {
    // Clamp incoming tuning targets to soft limits
    g_jointTargets[i] = constrain(msg->data.data[i], JOINT_MIN[i], JOINT_MAX[i]);
  }
}

// ── Error loop ──────────────────────────────────────────────────────────────
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

  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║   EVA ARM GESTURES — BOOT            ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.println("Firmware: arm_gestures v2.0 (with safety state machine)");

  // ── Ethernet micro-ROS transport ──
  Serial.println("[SETUP] Initializing micro-ROS Ethernet transport...");
  set_microros_transports();
  delay(2000);

  // ── CAN buses — init only, do NOT send any motor commands yet ──
  Serial.println("[SETUP] CAN1 (left arm)...");
  leftBus.begin(1000000);
  // leftBus.attachMotors(leftArm);

  Serial.println("[SETUP] CAN3 (right arm)...");
  rightBus.begin(1000000);
  rightBus.attachMotors(rightArm);

  // NOTE: setOrigin() is intentionally NOT called here.
  // Motors may not be powered yet. Origin is set in checkMotorReadiness()
  // once all motors confirm they are alive via CAN feedback.

  // ── RMCS motors — init bus only, calibrate later ──
  // calibrateEncoderPositionInDegrees() is deferred to checkMotorReadiness()
  // for the same reason: RMCS may not be powered when Teensy boots.
  Serial.println("[SETUP] RMCS I2C bus init...");
  rmcsLeft.begin(RMCS_ADDR);
  rmcsRight.begin(RMCS_ADDR);

  // ── Wrist servos — center immediately (safe neutral position) ──
  Serial.println("[SETUP] Wrist servos → center (90°)...");
  wristLeftServo.attach(SERVO_PIN_LEFT);
  wristRightServo.attach(SERVO_PIN_RIGHT);
  wristLeftServo.write(90);
  wristRightServo.write(90);

  // ── micro-ROS node + subscribers ──
  arm_state_msg.data.data     = cmd_buf;
  arm_state_msg.data.size     = 0;
  arm_state_msg.data.capacity = CMD_BUF_MAX;

  joint_targets_msg.data.data     = joint_data;
  joint_targets_msg.data.size     = 0;
  joint_targets_msg.data.capacity = NUM_JOINTS;

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_gesture_node", "", &support));

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

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &arm_state_sub, &arm_state_msg,
    &arm_state_callback, ON_NEW_DATA
  ));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &joint_targets_sub, &joint_targets_msg,
    &joint_targets_callback, ON_NEW_DATA
  ));

  // ── Start motor wait timer ──
  g_waitStartMs = millis();
  g_sysState    = SYS_WAITING;

  Serial.println("[SETUP] micro-ROS ready");
  Serial.println("  Node:   arm_gesture_node");
  Serial.println("  Topics: /arm_state (String), /joint_targets (Float32MultiArray)");
  Serial.println("[SYS] ══ WAITING for motors ══ (timeout 10s)");
  Serial.println("  Power motor supply now if not already on.");
  Serial.println("════════════════════════════════════════════\n");
}

// ── Serial reset command ────────────────────────────────────────────────────
// Typing "reset" + Enter in Serial monitor restarts the state machine from
// SYS_WAITING. Useful when motors lose power and come back without rebooting
// the Teensy. Works in any state.
void checkSerialReset() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      g_serialBuf[g_serialIdx] = '\0';
      if (g_serialIdx > 0 && strcmp(g_serialBuf, "reset") == 0) {
        Serial.println("\n[SYS] ══ RESET ══ — restarting motor detection");
        g_sysState    = SYS_WAITING;
        g_originSet   = false;
        g_waitStartMs = millis();
        g_gestureState = GESTURE_IDLE;
        g_currentGesture = nullptr;
        for (int i = 0; i < NUM_JOINTS; i++) g_jointTargets[i] = 0.0f;
        g_prevRmcsLeft  = -999.0f;
        g_prevRmcsRight = -999.0f;
        g_prevServoLeft  = -999.0f;
        g_prevServoRight = -999.0f;
        // Clear feedbackReceived so hasFeedback() returns false again
        for (auto &m : rightArm) m.clearFeedback();
        Serial.println("[SYS] ══ WAITING for motors ══");
        Serial.println("  Power motor supply now if not already on.");
      } else {
        // Not "reset" — signal checkSerialReady() that a line was entered
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
  // 1. Spin micro-ROS — process incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // 2. Read Serial — handle "reset" command (works in any state)
  checkSerialReset();

  // 3. Check if motors are ready (only active during SYS_WAITING)
  checkMotorReadiness();

  // 4. Live angle monitor + serial gate (only active during SYS_MONITORING)
  checkSerialReady();

  // 4. Update gesture interpolation (non-blocking)
  updateGesture();

  // 5. Push joint targets to hardware (blocked during SYS_WAITING and SYS_MONITORING)
  applyJointTargets();

  // 5. Monitor motor errors every loop tick (when ready)
  // if (g_sysState == SYS_READY) {
  //   for (auto &m : leftArm) {
  //     if (m.getErrorCode() != 0) {
  //       // Error already logged in handleCanMessage — no action taken in firmware
  //       // (motor has its own protections; we continue gestures unless stopped by operator)
  //     }
  //   }
  // }

  // 6. Periodic waiting log (every 2s)
  if (g_sysState == SYS_WAITING && (millis() - lastWaitLogMs >= 2000)) {
    lastWaitLogMs = millis();
    int ready = 0, expect = 0;
    // for (auto &m : leftArm)  { expect++; if (m.hasFeedback()) ready++; }
    for (auto &m : rightArm) { expect++; if (m.hasFeedback()) ready++; }
    uint32_t remaining = MOTOR_WAIT_TIMEOUT_MS - (millis() - g_waitStartMs);
    Serial.print("[SYS] Waiting for motors... ");
    Serial.print(ready); Serial.print("/"); Serial.print(expect);
    Serial.print(" online, timeout in ");
    Serial.print(remaining / 1000);
    Serial.println("s");
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
