#include "RMCS-220X_Right.h"

long RMCS220X_Right::stepsFromDegrees(float angle) {
    return angle/DEGREES_RESOLUTION;
}

double RMCS220X_Right::stepsToDegrees(long steps) {
    return steps*DEGREES_RESOLUTION;
}

void RMCS220X_Right::write2ByteAttr(byte command, int value) {
  Wire1.beginTransmission(i2cAddress);
  Wire1.write(byte(command));          // sends command byte
  Wire1.write(byte(lowByte(value)));   // sends value byte lsb
  Wire1.write(byte(highByte(value)));  // sends value byte msb
  Wire1.endTransmission();
}

void RMCS220X_Right::write4ByteAttr(byte command, long value) {
  byte buf[4];
  buf[0] = (byte) value;
  buf[1] = (byte) (value >> 8);
  buf[2] = (byte) (value >> 16);
  buf[3] = (byte) (value >> 24);
  
  Wire1.beginTransmission(i2cAddress);
  Wire1.write(byte(command));      // sends command byte
  
  Wire1.write(buf[0]);
  Wire1.write(buf[1]);
  Wire1.write(buf[2]);
  Wire1.write(buf[3]);
  
  Wire1.endTransmission();
}

int RMCS220X_Right::read2ByteAttr(byte command) {
    return (int) readAttr(command, 2);
}

long RMCS220X_Right::read4ByteAttr(byte command) {
    return readAttr(command, 4);
}

long RMCS220X_Right::readAttr(byte command, int numberOfBytes) {
  long result = 0;
  Wire1.beginTransmission(i2cAddress);
  Wire1.write(byte(command));          // send command byte
  Wire1.endTransmission();
  delay(5); // Delay to allow the motor's controller to react to the previous message
  Wire1.requestFrom((int) i2cAddress, (int) numberOfBytes);
  if (numberOfBytes <= Wire1.available()) { // if correct num of bytes received
    for(int i=0; i<numberOfBytes; i++){
      long currentByte = Wire1.read();
      currentByte = currentByte << (8*i);
      result |= currentByte;
    }
  }
  return result;
}

RMCS220X_Right::RMCS220X_Right() {
  // Empty constructor
}

void RMCS220X_Right::begin(byte incomingi2cAddress) {
    Wire1.begin(); // join i2c bus
    i2cAddress = incomingi2cAddress; // set motor address
}

void RMCS220X_Right::writeMaxSpeed(int maxSpeed) {
    write2ByteAttr(MAX_SPEED_ATTR, maxSpeed);
}

int RMCS220X_Right::readMaxSpeed() {
    return read2ByteAttr(MAX_SPEED_ATTR);
}

void RMCS220X_Right::writeSpeed(int motorSpeed) {
    write2ByteAttr(SPEED_ATTR, motorSpeed);
}

int RMCS220X_Right::readSpeed() {
    return read2ByteAttr(SPEED_ATTR);
}

void RMCS220X_Right::writeSpeedDamping(int value) {
    write2ByteAttr(SPEED_DAMP_ATTR, value);
}

int RMCS220X_Right::readSpeedDamping() {
    return read2ByteAttr(SPEED_DAMP_ATTR);
}

void RMCS220X_Right::writeSpeedFeedbackGainTerm(int value) {
    write2ByteAttr(SPEED_FDBK_GAIN_TERM_ATTR, value);
}

int RMCS220X_Right::readSpeedFeedbackGainTerm() {
    return read2ByteAttr(SPEED_FDBK_GAIN_TERM_ATTR);
}

void RMCS220X_Right::writePGainTerm(int value) {
    write2ByteAttr(P_GAIN_TERM_ATTR, value);
}

int RMCS220X_Right::readPGainTerm() {
    return read2ByteAttr(P_GAIN_TERM_ATTR);
}

void RMCS220X_Right::writeIGainTerm(int value) {
    write2ByteAttr(I_GAIN_TERM_ATTR, value);
}

int RMCS220X_Right::readIGainTerm() {
    return read2ByteAttr(I_GAIN_TERM_ATTR);
}

void RMCS220X_Right::calibrateEncoderPositionInSteps(long value) {
    write4ByteAttr(ENCODER_POS_ATTR, value);
}

int RMCS220X_Right::readEncoderPositionInSteps() {
    return read4ByteAttr(ENCODER_POS_ATTR);
}

void RMCS220X_Right::goToPositionInSteps(long value) {
    write4ByteAttr(GO_TO_POS_ATTR, value);
}

int RMCS220X_Right::readGoToPositionInSteps() {
    return read4ByteAttr(GO_TO_POS_ATTR);
}

void RMCS220X_Right::goToRelativePositionInSteps(long value) {
    write4ByteAttr(RELATIVE_GO_TO_ATTR, value);
}



void RMCS220X_Right::calibrateEncoderPositionInDegrees(double value) {
    calibrateEncoderPositionInSteps(stepsFromDegrees(value));
}

double RMCS220X_Right::readEncoderPositionInDegrees() {
    return stepsToDegrees(readEncoderPositionInSteps());
}

void RMCS220X_Right::goToPositionInDegrees(double value) {
    goToPositionInSteps(stepsFromDegrees(value));
}

double RMCS220X_Right::readGoToPositionInDegrees() {
    return stepsToDegrees(readGoToPositionInSteps());
}

void RMCS220X_Right::goToRelativePositionInDegrees(double value) {
    goToRelativePositionInSteps(stepsFromDegrees(value));
}


