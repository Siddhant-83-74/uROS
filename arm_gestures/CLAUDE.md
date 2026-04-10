# arm_gestures — Full Context for Claude

## What This Project Is

PlatformIO firmware for a **Teensy 4.1** that controls both arms (12 DOF) of the **EVA humanoid robot**.
- Talks to a **Jetson AGX Orin** running ROS2 Humble via **micro-ROS over native Ethernet (UDP)**
- Subscribes to `/arm_state` (String: gesture commands) and `/joint_targets` (Float32MultiArray: direct tuning)
- Motors are high-torque — all safety decisions in this codebase exist because of this

---

## Hardware — Exact Wiring

| Joint | Index | Motor Type | Bus | ID / Pin |
|-------|-------|-----------|-----|---------|
| L Shoulder Pitch | 0 | CubeMars AK | CAN1 | 0x01 |
| L Shoulder Roll  | 1 | CubeMars AK | CAN1 | 0x02 |
| L Bicep          | 2 | CubeMars AK | CAN1 | 0x03 |
| L Elbow          | 3 | CubeMars AK | CAN1 | 0x04 |
| L Forearm        | 4 | Rhino RMCS-220X | Wire (I2C0) | 0x08 |
| L Wrist          | 5 | PWM Servo | Teensy | Pin 10 |
| R Shoulder Pitch | 6 | CubeMars AK | CAN3 | 0x05 |
| R Shoulder Roll  | 7 | CubeMars AK | CAN3 | 0x06 |
| R Bicep          | 8 | CubeMars AK | CAN3 | 0x07 |
| R Elbow          | 9 | CubeMars AK | CAN3 | 0x08 |
| R Forearm        | 10 | Rhino RMCS-220X | Wire1 (I2C1) | 0x08 |
| R Wrist          | 11 | PWM Servo | Teensy | Pin 1 |

**CubeMars motor params:** control mode 6 (servo), speed 4000 ERPM, accel 6000 ERPM²
**Network:** Teensy IP `192.168.1.101`, micro-ROS agent (Jetson) `192.168.1.115:8880`

---

## File Structure

```
arm_gestures/
├── platformio.ini              # teensy41_eth environment
├── moveit_micro-ros.ino        # ALL application logic — read this first
├── CLAUDE.md                   # this file
├── README.md                   # hardware reference + workflow
├── src/
│   └── main.cpp                # micro-ROS Ethernet transport boilerplate (do not touch)
├── arm_tuner.py                # ROS2 Python keyboard tuner (run on Jetson)
└── lib/
    ├── CubeMars/
    │   ├── CubeMarsServoDual.h
    │   ├── CubeMarsServoDual.cpp
    │   └── CHANGES.md          # documents exactly what was added to this library
    ├── RMCS-220X_Left/
    │   ├── RMCS-220X_Left.h
    │   └── RMCS-220X_Left.cpp
    └── RMCS-220X_Right/
        ├── RMCS-220X_Right.h
        └── RMCS-220X_Right.cpp
```

---

## Build Commands

```bash
cd arm_gestures/

# Build
~/.platformio/penv/bin/pio run -e teensy41_eth

# Flash
~/.platformio/penv/bin/pio run -e teensy41_eth --target upload

# Serial monitor
~/.platformio/penv/bin/pio device monitor -b 115200
```

**CRITICAL:** Always use `~/.platformio/penv/bin/pio`, NEVER bare `pio`.
The conda base env has an unrelated `pio` package that crashes with `ModuleNotFoundError`.

---

## System State Machine — The Core Safety Design

The entire boot sequence exists to solve one problem: **CubeMars motors are high-torque. If Teensy boots before the motor PSU, it initializes targets to 0° and fires CAN commands every 10ms. When motors power on, they receive "go to 0°" immediately from wherever they physically are → violent dangerous jerk.**

### Three states

```
SYS_WAITING
  │ Poll hasFeedback() on all active CubeMars motors every 10ms.
  │ Zero CAN frames sent. Zero hardware touched.
  │ CubeMars motors auto-broadcast feedback in servo mode (no command needed).
  │ When all active motors have sent ≥1 feedback packet:
  ▼
SYS_MONITORING
  │ Still zero commands sent to any motor.
  │ Prints raw encoder angles (before origin is set) every 500ms to Serial.
  │ Operator reads the angles, verifies arm is in the right pose.
  │ When operator types any key + Enter:
  │   → setOrigin(0) called on each motor  [current physical pos = 0°]
  │   → delay(500ms) for motors to process origin command
  │   → RMCS calibrateEncoderPositionInDegrees(0)
  │   → g_jointTargets = {0,...,0}
  ▼
SYS_READY
  │ applyJointTargets() runs every loop tick.
  │ Motors receive 0° = "stay at current position" (origin was just set here).
  │ ROS2 commands from Jetson accepted.
  ▼ (stays here until power off)
```

### Key safety invariants

- `applyJointTargets()` returns immediately if state is `SYS_WAITING` or `SYS_MONITORING`
- `arm_state_callback` and `joint_targets_callback` both return early if not `SYS_READY`
- `setOrigin(0)` is called ONLY after motors are confirmed alive AND after operator confirms pose
- Angles are printed BEFORE setOrigin because after setOrigin they'd all show 0 (useless)
- `g_jointTargets` is seeded to 0 after setOrigin — 0° = current physical position = no movement

### Why setOrigin BEFORE first position command matters

`setOrigin(0)` is a CAN command. The motor processes it asynchronously. If we send a position command of 0° before the motor finishes processing setOrigin, the motor moves from its OLD origin to 0° → jerk. The `delay(500)` after setOrigin ensures the motor has processed it before any position is sent.

---

## Current Testing State (IMPORTANT — READ BEFORE CHANGING CODE)

**Right arm only.** Left arm is intentionally commented out in several places while testing. When you see lines like:
```cpp
// for (auto &m : leftArm) { ... }
// leftBus.attachMotors(leftArm);
```
These are deliberate — do NOT uncomment unless user asks to enable left arm.

### Lines currently commented out for right-arm testing

**In `setup()`:**
```cpp
// leftBus.attachMotors(leftArm);   // left arm CAN not connected
```

**In `applyJointTargets()`:**
```cpp
// for (int i = 0; i < 4; i++) {   // left arm CAN commands blocked
//   float safe = constrain(g_jointTargets[i], JOINT_MIN[i], JOINT_MAX[i]);
//   leftArm[i].setTargetPositionDeg(safe);
//   leftArm[i].update();
// }
```

**In `checkMotorReadiness()`:**
```cpp
// for (auto &m : leftArm) { expectCount++; if (m.hasFeedback()) readyCount++; }
```

**In `checkSerialReady()`:**
```cpp
// for (auto &m : leftArm) { if (m.hasFeedback()) { m.setOrigin(0); delay(5); } }
```

Because leftArm is not attached to any CAN ISR, `leftArm[i].hasFeedback()` is always false and `leftArm[i].getCurPosDeg()` returns 0.

---

## CubeMarsServoDual Library — What Was Changed

See `lib/CubeMars/CHANGES.md` for full details. Summary:

### Added `hasFeedback()` — motor aliveness detection
```cpp
// Header — private:
bool feedbackReceived;   // set true on first CAN feedback packet

// Header — public:
bool hasFeedback() const { return feedbackReceived; }

// .cpp — constructor initializer:
feedbackReceived(false),

// .cpp — handleCanMessage(), first line:
feedbackReceived = true;
```

### Added temperature + error decoding
Previously `cur_temp_c` and `cur_error` were declared but never populated (buf[6] and buf[7] were never read). Fixed in `handleCanMessage()`:
```cpp
cur_temp_c = msg.buf[6];
cur_error  = msg.buf[7];
if (cur_error != 0) { /* logs fault name to Serial */ }
```

### Full public API
```cpp
bool    hasFeedback()     const;  // true once first CAN reply received
float   getCurPosDeg()    const;  // degrees — updated every feedback packet
float   getCurSpeedErpm() const;  // electrical RPM
float   getCurCurrentA()  const;  // amps
uint8_t getTempC()        const;  // driver board °C  (now actually populated)
uint8_t getErrorCode()    const;  // 0=ok, see error table below
uint8_t getMotorId()      const;
void    setTargetPositionDeg(float deg);
void    setOrigin(uint8_t mode);  // 0=temporary, 1=permanent (survives power cycle)
void    update();                 // call every loop tick — pushes CAN frame
void    setSpeedErpm(int32_t);
void    setAccelErpm2(int32_t);
```

### CAN feedback packet format (manual §5.2)
```
buf[0:1]  int16  position    × 0.1  → degrees   (range ±3200°)
buf[2:3]  int16  speed       × 10   → ERPM
buf[4:5]  int16  current     × 0.01 → amps
buf[6]    uint8  temperature         → °C (driver board)
buf[7]    uint8  error code          → 0=ok, 1=overtemp, 2=overcurrent,
                                       3=overvolt, 4=undervolt, 5=encoder,
                                       6=phase-unbalance (hardware damaged)
```

### CAN command ID format
```
command_can_id = (control_mode << 8) | motor_id
```
For servo mode 6, motor 0x05: `command_can_id = 0x0605`

### setOrigin CAN command
```
msg.id  = (5 << 8) | motor_id   // CAN_PACKET_SET_ORIGIN_HERE = 5
msg.len = 1
buf[0]  = mode  // 0=temporary, 1=permanent
```

---

## Gesture Engine

### Data structures
```cpp
struct Keyframe {
  float    joints[12];    // target angle for each of 12 joints, degrees
  uint32_t duration_ms;   // time to interpolate FROM previous frame TO this frame
};

struct Gesture {
  Keyframe frames[10];    // up to 10 keyframes
  uint8_t  count;
};
```

### How interpolation works
- `updateGesture()` called every loop tick (non-blocking, millis-based)
- Uses **ease-out-quart**: `e = 1 - (1-t)^4` — fast start, smooth deceleration
- `g_frameStart[]` is snapshotted at the START of each keyframe (from current `g_jointTargets`)
- Interpolates from `g_frameStart` to `frame.joints` over `frame.duration_ms`
- On keyframe completion, advances to next frame or sets `GESTURE_IDLE`

### Current gestures (PLACEHOLDER ANGLES — need tuning)
- `GESTURE_HOME` — all zeros, 1000ms
- `GESTURE_HOME_SLOW` — all zeros, 3000ms (kept as reference, not used in boot anymore)
- `GESTURE_HELLO` — raises right arm, waves wrist 3 times, returns home
- `GESTURE_BYE` — raises both arms, waves both, returns home

### Adding a new gesture
1. Run `arm_tuner.py` on Jetson (keyboard control of all joints)
2. Press SPACE to print current pose, 1-9 to save waypoints, 0 to export full Gesture struct
3. Paste struct into `moveit_micro-ros.ino`
4. Add `strncmp` branch in `arm_state_callback`
5. Rebuild and flash

---

## Soft Position Limits

```cpp
const float JOINT_MIN[12] = {-90,-45,-90,-90,-90,-80,  -90,-45,-90,-90,-90,-80};
const float JOINT_MAX[12] = { 90, 45, 90, 90, 90, 80,   90, 45, 90, 90, 90, 80};
```

Applied via `constrain()` in `applyJointTargets()` to every motor before the CAN frame is sent.
Also applied to incoming `/joint_targets` ROS2 messages.
**These are conservative defaults — tighten them after physical testing.**

---

## RMCS-220X (I2C Forearm Motors) — Key Rules

- Has a blocking `delay(5)` inside read operations — never call in tight loops
- Only call `goToPositionInDegrees()` when target changes (`fabsf(new - prev) > 0.1f`)
- `calibrateEncoderPositionInDegrees(0)` sets current physical position as 0° — called in `checkSerialReady()` at the same time as CubeMars `setOrigin(0)`
- Left forearm on Wire (I2C0), right forearm on Wire1 (I2C1), both address 0x08

---

## PWM Servos (Wrists) — Key Rules

- Joint 5 (left wrist, pin 10): `servo.write(joint_deg + 90)` — 90° offset, center = 0°
- Joint 11 (right wrist, pin 1): same offset
- Only `write()` when value changes by >0.5° to avoid jitter
- Initialized to 90° (center) in `setup()` — servos are safe to move immediately on boot

---

## micro-ROS Patterns

### String message — always pre-allocate buffer
```cpp
char cmd_buf[32];
std_msgs__msg__String msg;
msg.data.data     = cmd_buf;
msg.data.size     = 0;
msg.data.capacity = sizeof(cmd_buf);
```
Omitting this causes silent crashes on receive.

### Float32MultiArray — same pattern
```cpp
float joint_data[12];
std_msgs__msg__Float32MultiArray msg;
msg.data.data     = joint_data;
msg.data.size     = 0;
msg.data.capacity = 12;
```

---

## Serial Monitor Boot Sequence

What you should see:

```
╔══════════════════════════════════════╗
║   EVA ARM GESTURES — BOOT            ║
╚══════════════════════════════════════╝
[SETUP] Initializing micro-ROS Ethernet transport...
[SETUP] CAN1 (left arm)...
[SETUP] CAN3 (right arm)...
[SETUP] RMCS I2C bus init...
[SETUP] Wrist servos → center (90°)...
[SETUP] micro-ROS ready
[SYS] ══ WAITING for motors ══ (timeout 10s)

[SYS] Waiting for motors... 0/4 online, timeout in 985s
[SYS] Waiting for motors... 0/4 online, timeout in 983s
   ← power the motor PSU now
[SYS] All 4 CubeMars motors confirmed online
[SYS] ══ MONITORING ══ — showing raw motor positions (origin NOT set yet)
[SYS] NO motor commands will be sent.
[SYS] Position the arm as needed, then type any key + Enter to SET ORIGIN and enable motion.

[ANGLES] R[ 12.50  -8.20  5.10  -3.40 ]  L[ ---.--  ---.--  ---.--  ---.-- ]  ← type any key + Enter
[ANGLES] R[ 12.50  -8.20  5.10  -3.40 ]  ...
   ← type anything + Enter in Serial monitor
[SYS] Setting origins at current physical position...
[SYS] Waiting for origin to settle...
[SYS] ══ READY ══ — origin set, holding at 0°, awaiting commands from Jetson
```

Notes:
- `---.--` = motor not attached (left arm commented out during testing)
- The angles shown in MONITORING are **raw encoder values before origin** — these are meaningful
- After typing Enter, origin is set and the arm holds that exact physical position
- `MOTOR_WAIT_TIMEOUT_MS = 1000000` (effectively infinite — user sets it manually)

---

## ROS2 Commands (from Jetson, only accepted in SYS_READY)

```bash
# Start micro-ROS agent on Jetson first:
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8880

# Gesture commands
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'hello'"
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'bye'"
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'home'"
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'stop'"   # abort gesture, hold position

# Direct joint tuning (interrupts any playing gesture)
ros2 topic pub --once /joint_targets std_msgs/msg/Float32MultiArray \
  "data: [0,0,0,0,0,0, -30,20,0,-45,0,0]"

# Interactive keyboard tuner (run on Jetson)
python3 arm_tuner.py
```

---

## Serial Monitor Commands (direct to Teensy, any state)

Format: `<hex_id>:<degrees>` or `<hex_id>:origin:<mode>`

```
05:45.0        → right shoulder pitch to 45°
R05:origin:1   → right shoulder pitch — permanent origin (survives power cycle)
06:origin:0    → right shoulder roll — temporary origin (lost on power cycle)
```

`L` prefix = search left arm motors only, `R` = right arm only, no prefix = search both.

---

## Hardware Safety Recommendation

Use **CubeMars Tool** PC app on each motor:
- Application Functions → **Timeout = 500 ms, Brake current = 2 A**
- Motors self-brake if no CAN frame received for 500ms (e.g. Teensy crash)
- This is a hardware-level safety net independent of firmware

---

## Permanent Origin Setup (One-Time, After Physical Testing)

When the arm is physically at the desired home position:
```
L01:origin:1   ← left shoulder pitch — permanent
L02:origin:1   ← left shoulder roll
L03:origin:1   ← left bicep
L04:origin:1   ← left elbow
R05:origin:1   ← right shoulder pitch
R06:origin:1   ← right shoulder roll
R07:origin:1   ← right bicep
R08:origin:1   ← right elbow
```
After this, the `setOrigin(0)` call in firmware becomes a confirmation rather than a requirement — motors remember their zero across power cycles.

---

## Bugs Fixed in This Session (Do Not Reintroduce)

### 1. Motor jerk on first position command after setOrigin
**Was:** `delay(100)` after `setOrigin(0)`, then immediately send 0°.
**Problem:** 100ms not enough — if position command arrived before motor processed setOrigin, motor moved from OLD origin to 0°.
**Fix:** `delay(500)` + angles printed BEFORE setOrigin (not after, where they'd all be 0).

### 2. readyCount checked against wrong total
**Was:** `allReady = (readyCount == 4)` hardcoded.
**Fix:** Dynamic `expectCount` incremented in the same loop that counts motors, so the threshold matches exactly how many motors are attached.

### 3. Origin set before operator verification
**Was:** setOrigin called immediately when motors detected, THEN angles printed (but they'd all show 0 after origin reset — useless).
**Fix:** MONITORING state shows raw angles BEFORE setOrigin. Operator confirms pose. THEN setOrigin is called on key press.

### 4. cur_temp_c and cur_error never populated
**Was:** `handleCanMessage()` decoded bytes 0–5 only. `getTempC()` and `getErrorCode()` always returned 0.
**Fix:** Added `cur_temp_c = msg.buf[6]` and `cur_error = msg.buf[7]` with fault logging.

---

## Open Items / Next Steps

- [ ] Tune `JOINT_MIN`/`JOINT_MAX` after physical testing — current ±90°/±45° are conservative
- [ ] Tune gesture keyframe angles in `GESTURE_HELLO` and `GESTURE_BYE` — current values are placeholders
- [ ] Enable left arm (uncomment leftBus.attachMotors and related lines) once right arm is confirmed working
- [ ] Configure hardware timeout on all motors via CubeMars Tool (500ms / 2A)
- [ ] Run permanent origin calibration (`XX:origin:1`) once home position is finalized
