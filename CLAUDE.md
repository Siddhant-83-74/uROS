# EVA Humanoid Robot — Firmware Monorepo

## Project Overview
Firmware for EVA, a humanoid robot. Each subsystem (arms, head, drive) is a separate PlatformIO project targeting **Teensy 4.1** with **micro-ROS over native Ethernet** to communicate with a **Jetson AGX Orin** running ROS2 (Humble).

---

## Repository Structure

```
arm_gestures/          — Arm gesture playback + keyboard tuning (both arms, 12 DOF)  ← ACTIVE
Arms_with_manual_training/ — Head animatronics (face, neck, eyes, jaw)
moveit_micro-ros_BothArms/ — MoveIt joint streaming for both arms
moveit_micro-ros_BothArms_Animatronics/ — Head animatronics (older version)
drive_eva_uros/        — Drive base control
platformio/            — Shared base PlatformIO config
```

Each project has:
- `platformio.ini` — board/transport config
- `src/main.cpp` — micro-ROS transport boilerplate (includes the .ino)
- `moveit_micro-ros.ino` — application logic
- `lib/` — local copies of motor libraries (never shared between projects)

---

## Hardware Map (Teensy 4.1)

### CubeMars AK Motors (CAN bus, servo mode 6)
- **CAN1**: Left arm — IDs 0x01–0x04 (shoulder pitch, shoulder roll, bicep, elbow)
- **CAN3**: Right arm — IDs 0x05–0x08 (mirror of left)
- Speed: 4000 ERPM, Acceleration: 6000 ERPM²
- Library: `lib/CubeMars/CubeMarsServoDual.h`

### Rhino RMCS-220X Motors (I2C)
- **Wire (I2C0)**: Left forearm, address 0x08
- **Wire1 (I2C1)**: Right forearm, address 0x08
- Library: `lib/RMCS-220X_Left/` and `lib/RMCS-220X_Right/`
- ⚠️ Has blocking `delay(5)` in read operations — avoid calling in tight loops
- Only call `goToPositionInDegrees()` when target changes — prevents I2C flooding

### PWM Servos
- **Pin 10**: Left wrist servo
- **Pin 1**: Right wrist servo
- **Pin 15**: Neck servo X (head projects)
- **Pin 14**: Neck servo Y (head projects)

### PCA9685 (Wire2, address 0x40) — Head only
- Channels 2–5: Eye servos
- Channel 14: Jaw servo
- Channel 15: Lip servo

---

## Build Commands

```bash
# Build for Teensy 4.1 with Ethernet transport
cd <project_dir>
~/.platformio/penv/bin/pio run -e teensy41_eth

# Upload (Teensy CLI uploader)
~/.platformio/penv/bin/pio run -e teensy41_eth --target upload

# Serial monitor
~/.platformio/penv/bin/pio device monitor -b 115200
```

**Important**: Always use `~/.platformio/penv/bin/pio`, NOT bare `pio`.
The conda base environment has an unrelated `pio` package that crashes with `ModuleNotFoundError`.

---

## Key Patterns

### micro-ROS String message — always pre-allocate buffer
```cpp
char cmd_buf[32];
std_msgs__msg__String msg;
msg.data.data     = cmd_buf;
msg.data.size     = 0;
msg.data.capacity = sizeof(cmd_buf);
```
Omitting this causes silent crashes on receive.

### Non-blocking control loop
Never use `delay()` in the main loop. Use `millis()`-based state machines:
- Gestures: see `updateGesture()` in `arm_gestures/moveit_micro-ros.ino`
- Head easing: see `updateNeckEasing()` in `Arms_with_manual_training`

### CubeMars motor update
```cpp
motor.setTargetPositionDeg(deg);
motor.update();   // call every loop tick — pushes CAN frame
```
The motor handles its own trajectory via ERPM speed/accel. Do NOT use `delay()` to wait for arrival.

### RMCS I2C — only write on change
```cpp
if (fabsf(newTarget - prevTarget) > 0.1f) {
  rmcs.goToPositionInDegrees(newTarget);
  prevTarget = newTarget;
}
```

---

## Safety — Power Sequencing (CubeMars Motors)

CubeMars AK motors are high-torque. **Never send position commands before confirming motors are powered and communicating.**

### The Problem
If Teensy boots before motors are powered:
- `g_jointTargets = {0,...,0}` by default
- Main loop sends "go to 0°" every 10ms
- When motor powers on it immediately executes "go to 0°" at full speed from wherever it physically is → **violent dangerous jerk**

### The Solution (implemented in arm_gestures)
Use a 2-state system machine driven by CAN feedback detection:

```
SYS_WAITING → poll hasFeedback() on all 8 motors, send NO CAN commands
                 (10s timeout — warns which motors didn't respond, proceeds anyway)
                 ↓  all 8 replied (or timeout)
              setOrigin(0) on each motor  ← defines current physical position as 0°
              g_jointTargets = {0, 0, ..., 0}
SYS_READY   → applyJointTargets() sends 0° every 10ms = motors hold current pose
              no movement until a command arrives from the Jetson
```

Key rules:
- `applyJointTargets()` returns immediately during `SYS_WAITING` — zero CAN frames sent
- Both ROS2 callbacks ignore commands until `SYS_READY`
- After `setOrigin(0)`, motor thinks it is at 0° — commanding 0° = stay exactly here, no movement
- `MOTOR_WAIT_TIMEOUT_MS` (default 10 000 ms) is the only tunable in this flow

### Hardware Timeout Safety Net
Configure via **CubeMars Tool** PC app on each motor:
- **Application Functions → Timeout = 500 ms, Brake current = 2 A**
- Motors will self-brake if no CAN frame received for 500ms (e.g. Teensy crash)

### Permanent Origin (One-Time Setup)
To persist zero across power cycles, with arm physically at home position, send via Serial:
```
01:origin:1   (Left shoulder pitch — permanent)
02:origin:1   (Left shoulder roll)
... etc.
```
After this, `setOrigin(0)` in firmware is no longer needed on every boot.

---

## CubeMars Feedback — Error Codes (manual §5.2)

Motors send error code in every CAN feedback packet. The library logs them automatically:

| Code | Meaning | Action |
|------|---------|--------|
| 0 | No fault | — |
| 1 | Over-temperature | Reduce load, let cool |
| 2 | Over-current | Check for mechanical block |
| 3 | Over-voltage | Check PSU |
| 4 | Under-voltage | Check PSU / battery |
| 5 | Encoder fault | Power cycle motor |
| 6 | Phase current unbalance | **Power off immediately — hardware may be damaged** |

---

## CubeMars Library — Key Methods

```cpp
// After calling update(), motor position is available:
float deg  = motor.getCurPosDeg();     // current position in degrees
float erpm = motor.getCurSpeedErpm();  // current speed in ERPM
float amps = motor.getCurCurrentA();   // current draw in amps
uint8_t t  = motor.getTempC();         // driver board temperature °C
uint8_t e  = motor.getErrorCode();     // 0 = ok (see table above)

// Safety check before first command:
bool alive = motor.hasFeedback();      // true once first CAN reply received
```

---

## Soft Position Limits (arm_gestures)

```cpp
// In moveit_micro-ros.ino — adjust after physical testing:
const float JOINT_MIN[12] = {-90,-45,-90,-90,-90,-80,  -90,-45,-90,-90,-90,-80};
const float JOINT_MAX[12] = { 90, 45, 90, 90, 90, 80,   90, 45, 90, 90, 90, 80};
```

Applied via `constrain()` in `applyJointTargets()` — protects against bad ROS2 messages.

---

## Networking
- Teensy Ethernet: MAC `AA:BB:CC:EE:DD:FF`, IP `192.168.1.101`
- micro-ROS Agent (Jetson): IP `192.168.1.115`, port `8880`

Start agent on Jetson:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8880
```

---

## Testing

```bash
# Wait for serial to show [SYS] ══ READY ══ before sending commands

# Gesture commands
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'hello'"
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'bye'"
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'home'"
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'stop'"

# Interactive joint tuning (run on Jetson)
python3 arm_gestures/arm_tuner.py
```

See `arm_gestures/README.md` for full gesture authoring workflow.
