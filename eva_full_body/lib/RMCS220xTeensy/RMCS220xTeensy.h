#ifndef RMCS220X_TEENSY_H
#define RMCS220X_TEENSY_H

#include <Arduino.h>
#include <Wire.h>

// Driver for the Rhino RMCS-220x encoder DC-servo motor over I2C.
//
// Teensy 4.x I2C buses and their default pins:
//   Wire  (bus 0) : SDA = pin 18, SCL = pin 19   <- default
//   Wire1 (bus 1) : SDA = pin 17, SCL = pin 16
//   Wire2 (bus 2) : SDA = pin 25, SCL = pin 24
//
// Pass the bus you wired as the second constructor argument:
//   RMCS220xTeensy motor(0x10);          // Wire  (pins 18/19)
//   RMCS220xTeensy motor(0x10, Wire1);   // Wire1 (pins 17/16)
//   RMCS220xTeensy motor(0x10, Wire2);   // Wire2 (pins 25/24)
//
// Hardware notes:
//   V+ : 11..15 VDC (external supply)
//   4.7k pull-ups on SDA and SCL to 3.3V are required.
//   Default I2C slave address: 0x10. I2C clock: 10..200 kHz.
//   Encoder: 1800 counts / revolution = 0.2 deg per count = 5 counts per deg.
//   All multi-byte values are little-endian signed integers.

class RMCS220xTeensy {
public:
    static constexpr uint8_t DEFAULT_ADDR     = 0x10;

    // I2C command byte table (from RMCS-220x datasheet)
    static constexpr uint8_t CMD_MAX_SPEED    = 0;  // 2-byte signed, 0..255
    static constexpr uint8_t CMD_SPEED        = 1;  // 2-byte signed, -255..+255
    static constexpr uint8_t CMD_DAMPING      = 2;  // 2-byte signed, 0..255
    static constexpr uint8_t CMD_ENCODER      = 3;  // 4-byte signed
    static constexpr uint8_t CMD_ABS_POSITION = 4;  // 4-byte signed
    static constexpr uint8_t CMD_GAIN_SPEED   = 5;  // 2-byte signed, 0..32767
    static constexpr uint8_t CMD_GAIN_P       = 6;  // 2-byte signed, 0..32767
    static constexpr uint8_t CMD_GAIN_I       = 7;  // 2-byte signed, 0..32767
    static constexpr uint8_t CMD_REL_POSITION = 8;  // 4-byte signed

    // 1800 counts / 360 deg
    static constexpr int32_t COUNTS_PER_DEG   = 5;

    RMCS220xTeensy(uint8_t slave_address = DEFAULT_ADDR, TwoWire &bus = Wire);

    // Initialises the I2C bus. clock_hz must be 10000..200000 per the spec.
    void begin(uint32_t clock_hz = 100000);

    // Velocity mode: signed -255..+255.
    void setSpeed(int16_t speed);

    // Tunables for the on-board servo loop.
    void setMaxSpeed(int16_t speed);   // 0..255, speed cap for G/R moves
    void setDamping(int16_t damping);  // 0..255
    void setGainP(int16_t gain);
    void setGainI(int16_t gain);
    void setGainSpeedFeedback(int16_t gain);

    // Position-servo mode.
    void writeAngle(float degrees);    // absolute, 0 == encoder 0
    void writeCounts(int32_t counts);  // absolute, raw counts
    void moveAngle(float degrees);     // relative to current position
    void moveCounts(int32_t counts);   // relative, raw counts

    // Encoder register.
    void resetEncoder();
    void writeEncoder(int32_t counts);
    int32_t readEncoder();
    int16_t readSpeed();

private:
    uint8_t _addr;
    TwoWire *_wire;

    void writeCmd16(uint8_t cmd, int16_t value);
    void writeCmd32(uint8_t cmd, int32_t value);
    bool readBytes(uint8_t cmd, uint8_t *buf, uint8_t len);
};

#endif
