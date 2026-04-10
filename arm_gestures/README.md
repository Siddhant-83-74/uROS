# arm_gestures — EVA Arm Gesture Firmware

PlatformIO project for Teensy 4.1 controlling both arms of the EVA humanoid robot (12 DOF total).
Subscribes to ROS2 topics from the Jetson AGX Orin over **native Ethernet** via micro-ROS.

---

## Hardware Overview

| Joint | Index | Actuator | Bus / Interface |
|-------|-------|----------|-----------------|
| L Shoulder Pitch  | 0  | CubeMars AK | CAN1, ID 0x01 |
| L Shoulder Roll   | 1  | CubeMars AK | CAN1, ID 0x02 |
| L Bicep           | 2  | CubeMars AK | CAN1, ID 0x03 |
| L Elbow           | 3  | CubeMars AK | CAN1, ID 0x04 |
| L Forearm         | 4  | Rhino RMCS-220X | Wire (I2C0), 0x08 |
| L Wrist           | 5  | PWM Servo | Teensy pin 10 |
| R Shoulder Pitch  | 6  | CubeMars AK | CAN3, ID 0x05 |
| R Shoulder Roll   | 7  | CubeMars AK | CAN3, ID 0x06 |
| R Bicep           | 8  | CubeMars AK | CAN3, ID 0x07 |
| R Elbow           | 9  | CubeMars AK | CAN3, ID 0x08 |
| R Forearm         | 10 | Rhino RMCS-220X | Wire1 (I2C1), 0x08 |
| R Wrist           | 11 | PWM Servo | Teensy pin 1 |

**Network:**
- Teensy IP: `192.168.1.101`
- micro-ROS Agent (Jetson): `192.168.1.115:8880`

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/arm_state` | `std_msgs/String` | Jetson → Teensy | Gesture command: `hello`, `bye`, `home`, `stop` |
| `/joint_targets` | `std_msgs/Float32MultiArray` | Jetson → Teensy | 12-float array for real-time joint tuning |

---

## Build & Flash

```bash
cd arm_gestures/

# Build
~/.platformio/penv/bin/pio run -e teensy41_eth

# Flash (Teensy CLI uploader required)
~/.platformio/penv/bin/pio run -e teensy41_eth --target upload

# Serial monitor
~/.platformio/penv/bin/pio device monitor -b 115200
```

---

## Safety System — Power Sequencing

### The Problem
CubeMars AK motors are extremely high torque. If the Teensy boots **before** motor power is applied:
- The firmware initializes `g_jointTargets = {0, 0, ..., 0}`
- The main loop immediately sends "go to 0°" CAN commands every 10ms
- When motors finally power on, they receive "go to 0°" and execute it **instantly** at full speed
- If a motor is physically at 60°, it jerks 60° in under a second — **dangerous**

### The Solution: 2-State System Machine

```
Boot
  │
  ▼
SYS_WAITING ─── No CAN commands sent, polling for motor feedback ──► (10s timeout)
  │                                                                         │
  │ All 8 motors replied with at least 1 CAN feedback packet               │
  ▼                                                                         ▼
  setOrigin(0) on each motor      ◄──────────────────────────────────────── ┘
  (current physical position IS now 0° for every motor)
  g_jointTargets = {0, 0, ..., 0}
  │
  ▼
SYS_READY ─── applyJointTargets() sends 0° every 10ms
              motors hold their current pose — NO movement
              arm stays still until a command arrives from the Jetson
```

**Key safety properties:**
- `applyJointTargets()` is a no-op during `SYS_WAITING` — zero CAN frames sent
- `arm_state_callback` ignores all commands until `SYS_READY`
- `joint_targets_callback` ignores all commands until `SYS_READY`
- After `setOrigin(0)`, motor encoder reads 0° at its current physical position — commanding 0° = hold exactly here, no jerk, no movement

### Serial Monitor During Boot

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
  Power motor supply now if not already on.

[SYS] Waiting for motors... 0/8 online, timeout in 10s
[SYS] Waiting for motors... 3/8 online, timeout in 8s
[SYS] All 8 CubeMars motors confirmed online
[SYS] Setting CubeMars origins (current position = 0°)...
[SYS] Calibrating RMCS encoders...
[SYS] ══ READY ══ — origins set, holding at 0°, awaiting commands from Jetson
```

---

## Soft Position Limits

Every joint target is clamped before being sent to hardware:

```cpp
const float JOINT_MIN[12] = {-90,-45,-90,-90,-90,-80,  -90,-45,-90,-90,-90,-80};
const float JOINT_MAX[12] = { 90, 45, 90, 90, 90, 80,   90, 45, 90, 90, 90, 80};
```

Applies to:
- Gesture keyframes (interpolated values)
- `joint_targets` ROS2 topic (tuning mode)
- All direct motor writes in `applyJointTargets()`

**Tune these limits after physical testing** — the defaults are conservative.

---

## Motor Error Monitoring

CubeMars AK motors report an error code in every CAN feedback packet (manual §5.2).
The library decodes this and logs it immediately:

```
[ERROR] M0x03 FAULT: OVER-TEMPERATURE (code 1)
[ERROR] M0x07 FAULT: OVER-CURRENT (code 2)
```

Error codes:

| Code | Meaning | Action |
|------|---------|--------|
| 0 | No fault | — |
| 1 | Over-temperature | Reduce duty cycle / let cool |
| 2 | Over-current | Check for mechanical block |
| 3 | Over-voltage | Check PSU |
| 4 | Under-voltage | Check PSU / battery |
| 5 | Encoder fault | Power cycle motor |
| 6 | Phase current unbalance | **Hardware likely damaged** — power off |

---

## Hardware Safety Recommendation

Use the **CubeMars Tool** PC application to configure a **CAN communication timeout** on each motor:

1. Connect motor via R-Link to PC
2. Go to **Application Functions**
3. Set **Timeout = 500 ms**
4. Set **Brake current = 2 A**
5. Save parameters

This means if the Teensy crashes or the CAN cable disconnects, motors automatically brake within 500ms — a hardware-level safety net independent of firmware.

---

## Gesture Workflow (Adding New Gestures)

### Step 1 — Find Safe Waypoints with arm_tuner.py

Run on the Jetson (micro-ROS agent must be running):
```bash
python3 arm_tuner.py
```

Keyboard controls:
```
q/a  → Joint 0  (L shoulder pitch)   up/down
w/s  → Joint 1  (L shoulder roll)    up/down
e/d  → Joint 2  (L bicep)            up/down
r/f  → Joint 3  (L elbow)            up/down
t/g  → Joint 4  (L forearm RMCS)     up/down
y/h  → Joint 5  (L wrist servo)      up/down
u/j  → Joint 6  (R shoulder pitch)   up/down
i/k  → Joint 7  (R shoulder roll)    up/down
o/l  → Joint 8  (R bicep)            up/down
p/;  → Joint 9  (R elbow)            up/down
[/'  → Joint 10 (R forearm RMCS)     up/down
]/\  → Joint 11 (R wrist servo)      up/down

+/-  → Increase/decrease step size
SPACE → Print current pose (C++ format)
1-9  → Save as waypoint N
0    → Export full Gesture struct (copy-paste to firmware)
z    → Zero all joints
ESC  → Exit
```

### Step 2 — Paste into Firmware

Press `0` in arm_tuner.py to get output like:
```cpp
const Gesture MY_GESTURE = {
  .frames = {
    { .joints = {-15.0, 10.0, 0.0, -20.0, 5.0, 10.0, -15.0, 10.0, 0.0, -20.0, 5.0, 10.0}, .duration_ms = 800 },
    { .joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, .duration_ms = 1000 },
  },
  .count = 2,
};
```

Add this to `moveit_micro-ros.ino` and add a branch in `arm_state_callback`.

### Step 3 — Rebuild and Flash

```bash
~/.platformio/penv/bin/pio run -e teensy41_eth --target upload
```

### Step 4 — Test

```bash
# On Jetson
ros2 topic pub --once /arm_state std_msgs/msg/String "data: 'hello'"
```

---

## Permanent Origin Calibration (One-Time Setup)

If you want motors to remember their zero point across power cycles:

1. Power everything on and let system reach `SYS_READY`
2. Run `arm_tuner.py`, press `z` to zero all joints
3. Physically position the arm at the desired "home" pose
4. Use Serial monitor to send origin commands per motor:
   ```
   01:origin:1    ← Left shoulder pitch — permanent zero
   02:origin:1    ← Left shoulder roll
   03:origin:1    ← Left bicep
   04:origin:1    ← Left elbow
   R05:origin:1   ← Right shoulder pitch
   R06:origin:1   ← Right shoulder roll
   R07:origin:1   ← Right bicep
   R08:origin:1   ← Right elbow
   ```
5. After this, `setOrigin(0)` in the firmware becomes optional — motors boot with the correct origin already saved (mode 1 = permanent, survives power cycle).

---

## File Structure

```
arm_gestures/
├── platformio.ini              # Board config: teensy41, native Ethernet
├── moveit_micro-ros.ino        # Main firmware: gesture engine + state machine
├── src/
│   └── main.cpp                # micro-ROS transport boilerplate
├── arm_tuner.py                # ROS2 Python keyboard tuning node (run on Jetson)
├── README.md                   # This file
└── lib/
    ├── CubeMars/
    │   ├── CubeMarsServoDual.h     # CAN bus template + motor class
    │   └── CubeMarsServoDual.cpp   # Motor commands + feedback decoding
    ├── RMCS-220X_Left/
    │   ├── RMCS-220X_Left.h
    │   └── RMCS-220X_Left.cpp      # Wire (I2C0) forearm driver
    └── RMCS-220X_Right/
        ├── RMCS-220X_Right.h
        └── RMCS-220X_Right.cpp     # Wire1 (I2C1) forearm driver
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Stuck at `WAITING for motors... 0/8` | Motors not powered | Turn on motor PSU |
| One motor never appears in feedback | CAN wiring issue or motor ID mismatch | Check CAN cable, verify IDs with CubeMars Tool |
| Motor jerks on first move | Origin not set correctly | Power arm at home position, send `XX:origin:1` per motor |
| `OVER-CURRENT` error during gesture | Gesture angle too aggressive or obstruction | Reduce angles, check for physical blockage |
| micro-ROS never connects | Ethernet not connected or wrong IP | Check cable, verify agent IP in platformio.ini |
| `RCCHECK FAIL` on boot | micro-ROS agent not running | Start agent: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8880` |
