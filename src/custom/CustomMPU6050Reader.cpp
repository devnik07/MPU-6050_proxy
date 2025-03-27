#include "CustomMPU6050Reader.h"

#include <Wire.h>
#include <Arduino.h>

CustomMPU6050Reader::CustomMPU6050Reader(IeepromMPU& ieepromManager, ComputationOption compOpt) 
                                        : eepromManager(ieepromManager), computationOption(compOpt) {
  gyroXOffset = 0;
  gyroYOffset = 0;
  gyroZOffset = 0;
  accXOffset = 0;
  accYOffset = 0;
  accZOffset = 0;
  prevTime = 0;
}

void CustomMPU6050Reader::init() {
  setupMPU();

  if (isCalibrated()) {
    loadCalibrationOffsets();
    prevTime = millis();
  } else {
    Serial.println("MPU not calibrated. Stopping execution.");
    while (true);   // Stop execution 
  }

}

void CustomMPU6050Reader::getRollPitchYaw(float& r, float& p, float& y) {
  float gForceX, gForceY, gForceZ, rateRoll, ratePitch, rateYaw;

  recordAccelRates(gForceX, gForceY, gForceZ);
  recordGyroRates(rateRoll, ratePitch, rateYaw);

  switch (computationOption) {
    case ComputationOption::ACC_RP:
      computeAccRP(r, p, gForceX, gForceY, gForceZ);
      break;
    case ComputationOption::ACC_RP_LPF:
      computeAccLpfRP(r, p, gForceX, gForceY, gForceZ);
      break;
    case ComputationOption::GYRO_RPY:
      computeGyroRPY(r, p, y, rateRoll, ratePitch, rateYaw);
      break;
    case ComputationOption::COMPL_RPY:
      computeComplRPY(r, p, y, gForceX, gForceY, gForceZ,
                      rateRoll, ratePitch, rateYaw);
      break;
    default:
      Serial.println("ERROR: Unexpected computation option");
      while (true); // Stop execution
  }
}

void CustomMPU6050Reader::getRotationQuaternion(float& w, float& x, float& y, float& z) {
  return;
}

void CustomMPU6050Reader::calibrate() {
  Serial.println("Starting calibration.");

  setupMPU();

  calibrateGyro();
  calibrateAccel();

  // Save offsets
  eepromManager.setXGyroOffset(gyroXOffset);
  eepromManager.setYGyroOffset(gyroYOffset);
  eepromManager.setZGyroOffset(gyroZOffset);
        
  eepromManager.setXAccOffset(accXOffset);
  eepromManager.setYAccOffset(accYOffset);
  eepromManager.setZAccOffset(accZOffset);

  eepromManager.setCalibrationFlag();

  Serial.println("Calibration completed.");
}

bool CustomMPU6050Reader::isCalibrated() {
  return eepromManager.getCalibrationFlag();
}

void CustomMPU6050Reader::resetCalibrationFlag() {
  eepromManager.resetCalibrationFlag();
  Serial.println("Calibration flag reset.");
}

void CustomMPU6050Reader::loadCalibrationOffsets() {
  Serial.println("Loading Calibration Offsets.");

  gyroXOffset = eepromManager.getXGyroOffset();
  gyroYOffset = eepromManager.getYGyroOffset();
  gyroZOffset = eepromManager.getZGyroOffset();

  accXOffset = eepromManager.getXAccOffset();
  accYOffset = eepromManager.getYAccOffset();
  accZOffset = eepromManager.getZAccOffset();

  Serial.print("Acc X: ");
  Serial.println(accXOffset);
  Serial.print("Acc Y: ");
  Serial.println(accYOffset);
  Serial.print("Acc Z: ");
  Serial.println(accZOffset);
  Serial.print("Gyro X: ");
  Serial.println(gyroXOffset);
  Serial.print("Gyro Y: ");
  Serial.println(gyroYOffset);
  Serial.print("Gyro Z: ");
  Serial.println(gyroZOffset);

  Serial.println("Calibration Offsets loaded.");

}

/* Stores the given value in the register located at the given address 
   using I2C data transmission.
*/
void CustomMPU6050Reader::writeTo(uint8_t address, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void CustomMPU6050Reader::setupMPU() {
  Wire.begin();
  wakeUp();

  setLowPassFilter();
  gyroConfig();
  accelConfig();
}

/* Power Management 1 Section 4.28 in Register Map datasheet
    Wakes up the MPU-6050 from sleep mode
*/
void CustomMPU6050Reader::wakeUp() {
  writeTo(0x6B, 0x00);
}


/* Configuration Section 4.3 in Register Map datasheet
    Register  Bit 7   Bit 6   Bit 5  Bit 4  Bit 3     Bit 2   Bit 1   Bit 0
   |   1A   |   -   |   -   |     EXT_SYNC_SET    |         DLPF_CFG        |
   (We're only interested in setting the DLPF_CFG [Digital Low Pass Filter Config])

   DLPF Configuration:
    DLPF_CFG  |    Accelerometer (F_s=1kHz)  |                Gyroscope               |
              | Bandwidth(Hz) |   Delay(ms)  | Bandwidth(Hz) |  Delay(ms)  |  Fs(kHz) |
   |    0     |     260       |      0       |     256       |    0.98     |     8    |
   |    1     |     184       |     2.0      |     188       |    1.90     |     1    |
   |    2     |      94       |     3.0      |      98       |    2.80     |     1    |
   |    3     |      44       |     4.9      |      42       |    4.80     |     1    |
   |    4     |      21       |     8.5      |      20       |    8.30     |     1    |
   |    5     |      10       |    13.8      |      10       |   13.40     |     1    |
   |    6     |       5       |    19.0      |       5       |   18.60     |     1    |
   |    7     |            RESERVED          |            RESERVED         |     8    |
*/
void CustomMPU6050Reader::setLowPassFilter() {
  switch (DLPF_CFG) {
    case 0:
      writeTo(0x1A, 0x00);
      break;
    case 1:
      writeTo(0x1A, 0x01);
      break;
    case 2:
      writeTo(0x1A, 0x02);
      break;
    case 3:
      writeTo(0x1A, 0x03);
      break;
    case 4:
      writeTo(0x1A, 0x04);
      break;
    case 5:
      writeTo(0x1A, 0x05);
      break;
    case 6:
      writeTo(0x1A, 0x06);
      break;
    default:
      writeTo(0x1A, 0x00);
  }
}

/* Gyroscope Configuration Section 4.4 in Register Map datasheet
    Register  Bit 7   Bit 6   Bit 5     Bit 4  Bit 3     Bit 2   Bit 1   Bit 0
   |   1B   | XG_ST | YG_ST | ZG_ST | Full Scale Range |   -   |   -   |   -   |
   |        |       |       |       |   Select FS_SEL  |       |       |       |
   (Setting XG_ST, YG_ST or ZG_ST to 1 will perform a self test)

   Gyroscope Full Scale Range Selection:
    FS_SEL   Full Scale Range   LSB Sensitivity
   |   0   |   +/-  250 °/s   |      131.0      |
   |   1   |   +/-  500 °/s   |       65.5      |
   |   2   |   +/- 1000 °/s   |       32.8      |
   |   3   |   +/- 2000 °/s   |       16.4      |

   --> Note: FS_SEL/360 * 60sec/min = RotationValue in RPM (Rotations Per Minute)
*/
void CustomMPU6050Reader::gyroConfig() {
  switch (GY_RANGE) {
    case 0:
      writeTo(0x1B, 0x00);
      gyroLSBSens = 131.0;
      break;
    case 1:
      writeTo(0x1B, 0x08);
      gyroLSBSens = 65.5;
      break;
    case 2:
      writeTo(0x1B, 0x10);
      gyroLSBSens = 32.8;
      break;
    case 3:
      writeTo(0x1B, 0x18);
      gyroLSBSens = 16.4;
      break;
    default:
      writeTo(0x1B, 0x00);
      gyroLSBSens = 131.0;
  }
}

/* Accelerometer Configuration Section 4.5 in the Register Map datasheet
    Register  Bit 7   Bit 6   Bit 5     Bit 4  Bit 3     Bit 2   Bit 1   Bit 0
   |   1C   | XA_ST | YA_ST | ZA_ST | Full Scale Range |   -   |   -   |   -   |
   |        |       |       |       |  Select AFS_SEL  |       |       |       |
   (Setting XA_ST, YA_ST or ZA_ST to 1 will perform a self test)

   Accelerometer Full Scale Range Selection:
    FS_SEL   Full Scale Range   LSB Sensitivity
   |   0   |     +/-   2g     |      16384      |
   |   1   |     +/-   4g     |       8192      |
   |   2   |     +/-   8g     |       4096      |
   |   3   |     +/-  16g     |       2048      |
*/
void CustomMPU6050Reader::accelConfig() {
  switch (AC_RANGE) {
    case 0:
      writeTo(0x1C, 0x00);
      accLSBSens = 16384.0;
      break;
    case 1:
      writeTo(0x1C, 0x08);
      accLSBSens = 8192.0;
      break;
    case 2:
      writeTo(0x1C, 0x10);
      accLSBSens = 4096.0;
      break;
    case 3:
      writeTo(0x1C, 0x18);
      accLSBSens = 2048.0;
      break;
    default:
      writeTo(0x1C, 0x00);
      accLSBSens = 16384.0;
  }
}

/*
  Computes offset values for the Gyroscope.
*/
void CustomMPU6050Reader::calibrateGyro() {
  Serial.println("Calibrating Gyroscope ...");

  int numSamples = 2000;
  int16_t gyX, gyY, gyZ;

  for (int i = 0; i < numSamples; i++) {
    readGyroData(gyX, gyY, gyZ);
    gyroXOffset += gyX;
    gyroYOffset += gyY;
    gyroZOffset += gyZ;
    delay(1);
  }

  gyroXOffset /= numSamples;
  gyroYOffset /= numSamples;
  gyroZOffset /= numSamples;

  Serial.println("Gyroscope Calibration completed.");
}

/*
  Compute offset values for the Accelerometer.
*/
void CustomMPU6050Reader::calibrateAccel() {
  Serial.println("Calibrating Accelerometer ...");

  int numSamples = 2000;
  int16_t acX, acY, acZ;

  for (int i = 0; i < numSamples; i++) {
    readAccelData(acX, acY, acZ);
    accXOffset += acX;
    accYOffset += acY;
    accZOffset += acZ;
    delay(1);
  }

  accXOffset /= numSamples;
  accYOffset /= numSamples;
  accZOffset = (accZOffset / numSamples) - accLSBSens;

  Serial.println("Accelerometer Calibration completed.");
}

/* Gyroscope Measurements Section 4.19 in the Register Map datasheet
    Measurements are stored in registers 43 - 48.
    Note: each measurement is broken up into High and Low bytes.
*/
void CustomMPU6050Reader::readGyroData(int16_t& gyX, int16_t& gyY, int16_t& gyZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);                     // starting with register 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // request gyroscope measurements
  gyX = Wire.read()<<8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyY = Wire.read()<<8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyZ = Wire.read()<<8 | Wire.read();   // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

/* 
   Computation of exerted roll, pitch and yaw rates from gyroscope measurements.
*/
void CustomMPU6050Reader::recordGyroRates(float& rateRoll, float& ratePitch, float& rateYaw) {
  int16_t gyX, gyY, gyZ;
  readGyroData(gyX, gyY, gyZ);
  if (isCalibrated()) {
    gyX -= gyroXOffset;
    gyY -= gyroYOffset;
    gyZ -= gyroZOffset;
  }
  processGyroData(rateRoll, ratePitch, rateYaw, gyX, gyY, gyZ);
}

/* Conversion of the raw Gyroscope measurements into exerted roll, pitch and yaw rates
    Section 4.19 Gyroscope Measurements
     
     FS_SEL   Full Scale Range    LSB Sensitivity
   |   0    |  +-  250°/s      |   131.0 LSB/°/s   |
   |   1    |  +-  500°/s      |    65.5 LSB/°/s   |
   |   2    |  +- 1000°/s      |    32.8 LSB/°/s   |
   |   3    |  +- 2000°/s      |    16.4 LSB/°/s   |

    --> Note: directional rotational force [°/s] = gyX/LSB Sensitivity
*/
void CustomMPU6050Reader::processGyroData(float& rateRoll, float& ratePitch, float& rateYaw,
                                          int16_t gyX, int16_t gyY, int16_t gyZ) {
  rateRoll = (float)(gyX) / gyroLSBSens;
  ratePitch = (float)(gyY) / gyroLSBSens;
  rateYaw = (float)(gyZ) / gyroLSBSens;
}

/* Accelerometer Measurements Section 4.17 in the Register Map datasheet
    Measurements are stored in registers 3B - 40.
    Note: each measurement is broken up into High and Low bytes.
*/
void CustomMPU6050Reader::readAccelData(int16_t& acX, int16_t& acY, int16_t& acZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // request acceleration measurements
  acX = Wire.read()<<8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acY = Wire.read()<<8 | Wire.read();   // Ox3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acZ = Wire.read()<<8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

/*
   Computation of exerted g forces from accelerometer measurements.
*/
void CustomMPU6050Reader::recordAccelRates(float& gForceX, float& gForceY, float& gForceZ) {
  int16_t acX, acY, acZ;
  readAccelData(acX, acY, acZ);
  if (isCalibrated()) {
    acX -= accXOffset;
    acY -= accYOffset;
    acZ -= accZOffset;
  }
  processAccelData(gForceX, gForceY, gForceZ, acX, acY, acZ);
}

/* Conversion of the raw Accelerometer measurements into exerted g forces
    Section 4.17 Accelerometer Measurements
     
    AFS_SEL   Full Scale Range   LSB Sensitivity
   |   0    |      +-  2g      |   16384 LSB/g   |
   |   1    |      +-  4g      |    8192 LSB/g   |
   |   2    |      +-  8g      |    4096 LSB/g   |
   |   3    |      +- 16g      |    2048 LSB/g   |

    --> Note: directional g force = acX/LSB Sensitivity
*/
void CustomMPU6050Reader::processAccelData(float& gForceX, float& gForceY, float& gForceZ,
                                           int16_t acX, int16_t acY, int16_t acZ) {
  gForceX = (float)(acX) / accLSBSens;
  gForceY = (float)(acY) / accLSBSens;
  gForceZ = (float)(acZ) / accLSBSens;
}

/*
  Default roll and pitch computations using only accelerometer.
*/
void CustomMPU6050Reader::computeAccRP(float& r, float& p, float gForceX, float gForceY, float gForceZ) {
  r = atan2(gForceY, sqrt(pow(gForceX, 2) + pow(gForceZ, 2))) * 180/PI;
  p = atan2(gForceX, sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180/PI;
}

/*
  Low-pass filter roll and pitch computations using only accelerometer.
*/
void CustomMPU6050Reader::computeAccLpfRP(float& r, float& p, float gForceX, float gForceY, float gForceZ) {
  r = ALPHA_LPF * r + (1 - ALPHA_LPF) * atan2(gForceY, sqrt(pow(gForceX, 2) + pow(gForceZ, 2))) * 180/PI;
  p = ALPHA_LPF * p + (1 - ALPHA_LPF) * atan2(gForceX, sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180/PI;
}

/*
  Default roll, pitch and yaw (rpy) computations using only gyroscope.
*/
void CustomMPU6050Reader::computeGyroRPY(float& r, float& p, float& y,
                                         float rateRoll, float ratePitch, float rateYaw) {
  float currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  r += rateRoll * deltaTime;
  p += (-1) * ratePitch * deltaTime;
  y += (-1) * rateYaw * deltaTime;
}

/*
  Complementary filter roll, pitch and yaw computations using sensor fusion
  of gyroscope and accelerometer measurements.
*/
void CustomMPU6050Reader::computeComplRPY(float& r, float& p, float& y,
                                          float gForceX, float gForceY, float gForceZ,
                                          float rateRoll, float ratePitch, float rateYaw) {
  float accRoll, accPitch;
  computeAccRP(accRoll, accPitch, gForceX, gForceY, gForceZ);
  float currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  r = ALPHA_COMPL_FILTER * (r + rateRoll * deltaTime);
  r += (1 - ALPHA_COMPL_FILTER) * accRoll;
  p = ALPHA_COMPL_FILTER * (p + (-1) * ratePitch * deltaTime);
  p += (1 - ALPHA_COMPL_FILTER) * accPitch;
  y += rateYaw * (-1)* deltaTime;
}
