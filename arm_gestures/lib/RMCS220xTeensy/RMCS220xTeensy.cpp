#include "RMCS220xTeensy.h"

RMCS220xTeensy::RMCS220xTeensy(uint8_t slave_address, TwoWire &bus)
    : _addr(slave_address), _wire(&bus) {}

void RMCS220xTeensy::begin(uint32_t clock_hz) {
    if (clock_hz <  10000) clock_hz =  10000;
    if (clock_hz > 200000) clock_hz = 200000;
    _wire->begin();
    _wire->setClock(clock_hz);
}

void RMCS220xTeensy::writeCmd16(uint8_t cmd, int16_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(cmd);
    _wire->write((uint8_t)(value & 0xFF));
    _wire->write((uint8_t)((value >> 8) & 0xFF));
    _wire->endTransmission();
}

void RMCS220xTeensy::writeCmd32(uint8_t cmd, int32_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(cmd);
    _wire->write((uint8_t)(value & 0xFF));
    _wire->write((uint8_t)((value >> 8) & 0xFF));
    _wire->write((uint8_t)((value >> 16) & 0xFF));
    _wire->write((uint8_t)((value >> 24) & 0xFF));
    _wire->endTransmission();
}

bool RMCS220xTeensy::readBytes(uint8_t cmd, uint8_t *buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(cmd);
    if (_wire->endTransmission(false) != 0) return false;
    uint8_t got = _wire->requestFrom((int)_addr, (int)len);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _wire->read();
    return true;
}

void RMCS220xTeensy::setSpeed(int16_t speed) {
    if (speed >  255) speed =  255;
    if (speed < -255) speed = -255;
    writeCmd16(CMD_SPEED, speed);
}

void RMCS220xTeensy::setMaxSpeed(int16_t speed) {
    if (speed < 0)   speed = 0;
    if (speed > 255) speed = 255;
    writeCmd16(CMD_MAX_SPEED, speed);
}

void RMCS220xTeensy::setDamping(int16_t damping) {
    if (damping < 0)   damping = 0;
    if (damping > 255) damping = 255;
    writeCmd16(CMD_DAMPING, damping);
}

void RMCS220xTeensy::setGainP(int16_t gain)             { writeCmd16(CMD_GAIN_P, gain); }
void RMCS220xTeensy::setGainI(int16_t gain)             { writeCmd16(CMD_GAIN_I, gain); }
void RMCS220xTeensy::setGainSpeedFeedback(int16_t gain) { writeCmd16(CMD_GAIN_SPEED, gain); }

void RMCS220xTeensy::writeAngle(float degrees) {
    writeCounts((int32_t)(degrees * COUNTS_PER_DEG));
}

void RMCS220xTeensy::writeCounts(int32_t counts) {
    writeCmd32(CMD_ABS_POSITION, counts);
}

void RMCS220xTeensy::moveAngle(float degrees) {
    moveCounts((int32_t)(degrees * COUNTS_PER_DEG));
}

void RMCS220xTeensy::moveCounts(int32_t counts) {
    writeCmd32(CMD_REL_POSITION, counts);
}

void RMCS220xTeensy::resetEncoder() {
    writeEncoder(0);
}

void RMCS220xTeensy::writeEncoder(int32_t counts) {
    writeCmd32(CMD_ENCODER, counts);
}

int32_t RMCS220xTeensy::readEncoder() {
    uint8_t b[4];
    if (!readBytes(CMD_ENCODER, b, 4)) return 0;
    return (int32_t)((uint32_t)b[0]        |
                     ((uint32_t)b[1] <<  8) |
                     ((uint32_t)b[2] << 16) |
                     ((uint32_t)b[3] << 24));
}

int16_t RMCS220xTeensy::readSpeed() {
    uint8_t b[2];
    if (!readBytes(CMD_SPEED, b, 2)) return 0;
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}
