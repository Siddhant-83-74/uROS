#include "RMCS-220X_Left.h"

long RMCS220X_Left::stepsFromDegrees(float angle) {
    return angle/DEGREES_RESOLUTION;
}

double RMCS220X_Left::stepsToDegrees(long steps) {
    return steps*DEGREES_RESOLUTION;
}

void RMCS220X_Left::write2ByteAttr(byte command, int value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(command));          // sends command byte
  Wire.write(byte(lowByte(value)));   // sends value byte lsb
  Wire.write(byte(highByte(value)));  // sends value byte msb
  Wire.endTransmission();
}

void RMCS220X_Left::write4ByteAttr(byte command, long value) {
  byte buf[4];
  buf[0] = (byte) value;
  buf[1] = (byte) (value >> 8);
  buf[2] = (byte) (value >> 16);
  buf[3] = (byte) (value >> 24);
  
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(command));      // sends command byte
  
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.write(buf[2]);
  Wire.write(buf[3]);
  
  Wire.endTransmission();
}

int RMCS220X_Left::read2ByteAttr(byte command) {
    return (int) readAttr(command, 2);
}

long RMCS220X_Left::read4ByteAttr(byte command) {
    return readAttr(command, 4);
}

long RMCS220X_Left::readAttr(byte command, int numberOfBytes) {
  long result = 0;
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(command));          // send command byte
  Wire.endTransmission();
  delay(5); // Delay to allow the motor's controller to react to the previous message
  Wire.requestFrom((int) i2cAddress, (int) numberOfBytes);
  if (numberOfBytes <= Wire.available()) { // if correct num of bytes received
    for(int i=0; i<numberOfBytes; i++){
      long currentByte = Wire.read();
      currentByte = currentByte << (8*i);
      result |= currentByte;
    }
  }
  return result;
}

RMCS220X_Left::RMCS220X_Left() {
  // Empty constructor
}

void RMCS220X_Left::begin(byte incomingi2cAddress) {
    Wire.begin(); // join i2c bus
    i2cAddress = incomingi2cAddress; // set motor address
}

void RMCS220X_Left::writeMaxSpeed(int maxSpeed) {
    write2ByteAttr(MAX_SPEED_ATTR, maxSpeed);
}

int RMCS220X_Left::readMaxSpeed() {
    return read2ByteAttr(MAX_SPEED_ATTR);
}

void RMCS220X_Left::writeSpeed(int motorSpeed) {
    write2ByteAttr(SPEED_ATTR, motorSpeed);
}

int RMCS220X_Left::readSpeed() {
    return read2ByteAttr(SPEED_ATTR);
}

void RMCS220X_Left::writeSpeedDamping(int value) {
    write2ByteAttr(SPEED_DAMP_ATTR, value);
}

int RMCS220X_Left::readSpeedDamping() {
    return read2ByteAttr(SPEED_DAMP_ATTR);
}

void RMCS220X_Left::writeSpeedFeedbackGainTerm(int value) {
    write2ByteAttr(SPEED_FDBK_GAIN_TERM_ATTR, value);
}

int RMCS220X_Left::readSpeedFeedbackGainTerm() {
    return read2ByteAttr(SPEED_FDBK_GAIN_TERM_ATTR);
}

void RMCS220X_Left::writePGainTerm(int value) {
    write2ByteAttr(P_GAIN_TERM_ATTR, value);
}

int RMCS220X_Left::readPGainTerm() {
    return read2ByteAttr(P_GAIN_TERM_ATTR);
}

void RMCS220X_Left::writeIGainTerm(int value) {
    write2ByteAttr(I_GAIN_TERM_ATTR, value);
}

int RMCS220X_Left::readIGainTerm() {
    return read2ByteAttr(I_GAIN_TERM_ATTR);
}

void RMCS220X_Left::calibrateEncoderPositionInSteps(long value) {
    write4ByteAttr(ENCODER_POS_ATTR, value);
}

int RMCS220X_Left::readEncoderPositionInSteps() {
    return read4ByteAttr(ENCODER_POS_ATTR);
}

void RMCS220X_Left::goToPositionInSteps(long value) {
    write4ByteAttr(GO_TO_POS_ATTR, value);
}

int RMCS220X_Left::readGoToPositionInSteps() {
    return read4ByteAttr(GO_TO_POS_ATTR);
}

void RMCS220X_Left::goToRelativePositionInSteps(long value) {
    write4ByteAttr(RELATIVE_GO_TO_ATTR, value);
}



void RMCS220X_Left::calibrateEncoderPositionInDegrees(double value) {
    calibrateEncoderPositionInSteps(stepsFromDegrees(value));
}

double RMCS220X_Left::readEncoderPositionInDegrees() {
    return stepsToDegrees(readEncoderPositionInSteps());
}

void RMCS220X_Left::goToPositionInDegrees(double value) {
    goToPositionInSteps(stepsFromDegrees(value));
}

double RMCS220X_Left::readGoToPositionInDegrees() {
    return stepsToDegrees(readGoToPositionInSteps());
}

void RMCS220X_Left::goToRelativePositionInDegrees(double value) {
    goToRelativePositionInSteps(stepsFromDegrees(value));
}


