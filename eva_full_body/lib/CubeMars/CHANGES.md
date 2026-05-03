# CubeMarsServoDual — Library Changes

Changes made to the original CubeMarsServoDual library for the EVA arm_gestures project.

---

## Added: Motor aliveness detection — `hasFeedback()`

**Files:** `CubeMarsServoDual.h`, `CubeMarsServoDual.cpp`

**Why:** CubeMars AK motors are high-torque. If the Teensy boots before the motor PSU is on,
sending position commands into the void is harmless — but the moment motors power on, they
receive the queued "go to 0°" command and execute it instantly at full speed from whatever
physical position they are at. This is dangerous.

The fix requires knowing *when* a motor has actually powered on and is communicating.
CubeMars motors in servo mode broadcast CAN feedback packets automatically (no command
needed). The first feedback packet proves the motor is alive and its encoder is valid.

**What was added:**

### `CubeMarsServoDual.h` — private field
```cpp
bool feedbackReceived;   // set true on first CAN feedback packet received
```

### `CubeMarsServoDual.h` — public getter
```cpp
// Returns true once this motor has sent at least one CAN feedback packet.
// Use before issuing the first position command to confirm the motor is
// powered and communicating — prevents dangerous startup jerks.
bool hasFeedback() const { return feedbackReceived; }
```

### `CubeMarsServoDual.cpp` — constructor initializer
```cpp
feedbackReceived(false),   // added to initializer list
```

### `CubeMarsServoDual.cpp` — set in `handleCanMessage()`
```cpp
// Motor is alive — mark as ready for position commands
feedbackReceived = true;   // added at top of handleCanMessage()
```

---

## Added: Temperature and error decoding in `handleCanMessage()`

**Files:** `CubeMarsServoDual.h`, `CubeMarsServoDual.cpp`

**Why:** The original `handleCanMessage()` decoded position, speed, and current from the
CAN feedback packet (bytes 0–5) but left `buf[6]` (temperature) and `buf[7]` (error code)
unread. The fields `cur_temp_c` and `cur_error` were declared in the class but never populated,
so `getTempC()` and `getErrorCode()` always returned 0 (including during real motor faults).

**What was added:**

### `CubeMarsServoDual.h` — public getters (these existed but now return real values)
```cpp
uint8_t getTempC()     const { return cur_temp_c; }   // driver board °C
uint8_t getErrorCode() const { return cur_error;   }   // 0 = ok, see table below
```

### `CubeMarsServoDual.cpp` — decoding in `handleCanMessage()`
```cpp
cur_temp_c = msg.buf[6];   // driver board temperature °C
cur_error  = msg.buf[7];   // 0=ok, 1=overtemp, 2=overcurrent, 3=overvolt,
                            //       4=undervolt, 5=encoder, 6=phase-unbalance

if (cur_error != 0) {
  // logs fault name + code to Serial immediately
}
```

### Error code table (CubeMars manual §5.2)

| Code | Meaning | Recommended Action |
|------|---------|-------------------|
| 0 | No fault | — |
| 1 | Over-temperature | Reduce duty cycle, let motor cool |
| 2 | Over-current | Check for mechanical obstruction |
| 3 | Over-voltage | Check PSU voltage |
| 4 | Under-voltage | Check PSU / battery charge |
| 5 | Encoder fault | Power cycle the motor |
| 6 | Phase current unbalance | **Power off immediately — hardware may be damaged** |

---

## Summary of all new public API

```cpp
// Aliveness check — call before first position command
bool    hasFeedback()  const;   // true once first CAN feedback received

// Feedback values — updated every time motor sends a CAN packet
float   getCurPosDeg()    const;   // current position in degrees  (was present, unchanged)
float   getCurSpeedErpm() const;   // electrical RPM               (was present, unchanged)
float   getCurCurrentA()  const;   // current draw in amps         (was present, unchanged)
uint8_t getTempC()        const;   // driver board temperature °C  (now returns real value)
uint8_t getErrorCode()    const;   // 0 = ok (see table above)     (now returns real value)
```

No existing methods were removed or renamed. All changes are backwards-compatible.
