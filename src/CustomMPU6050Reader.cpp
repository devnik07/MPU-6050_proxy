#include "CustomMPU6050Reader.h"

#include <Wire.h>
#include <Arduino.h>

CustomMPU6050Reader::CustomMPU6050Reader(ComputationOption compOpt) : computationOption(compOpt) {
  gyroXOffset = 0;
  gyroYOffset = 0;
  gyroZOffset = 0;
  accXOffset = 0;
  accYOffset = 0;
  accZOffset = 0;
  calibrated = false;
}

void CustomMPU6050Reader::init() {
  Wire.begin();
  wakeUp();
  setLowPassFilter();
  gyroConfig();
  accelConfig();
  calibrateGyro();
  calibrateAccel();
  calibrated = true;
}

void CustomMPU6050Reader::getRollPitchYaw(float& r, float& p, float& y) {
  float gForceX, gForceY, gForceZ, rateRoll, ratePitch, rateYaw;
  float curr_time, delta_time;
  float prev_time = 0;

  recordAccelData(gForceX, gForceY, gForceZ);
  recordGyroData(rateRoll, ratePitch, rateYaw);

  switch (computationOption) {
    case ComputationOption::ACC_RP:
      computeAccRP(r, p, gForceX, gForceY, gForceZ);
      break;
    case ComputationOption::ACC_RP_LPF:
      computeAccLpfRP(r, p, gForceX, gForceY, gForceZ);
      break;
    case ComputationOption::GYRO_RPY:
      computeGyroRPY(r, p, y, rateRoll, ratePitch, rateYaw,
                     curr_time, delta_time, prev_time);
      break;
    case ComputationOption::COMPL_RPY:
      computeComplRPY(r, p, y, gForceX, gForceY, gForceZ,
                      rateRoll, ratePitch, rateYaw,
                      curr_time, delta_time, prev_time);
      break;
    default:
      Serial.println("ERROR: Unexpected computation option");
      while (true); // Stop execution
  }
}

/*! Power Management 1 Section 4.28 in Register Map datasheet
    Wakes up the MPU-6050 from sleep mode
*/
void CustomMPU6050Reader::wakeUp() {
  writeTo(0x6B, 0x00);
}

void CustomMPU6050Reader::writeTo(uint8_t address, uint8_t value) {
  // TODO: write documentation for this method
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

/*! Configuration Section 4.3 in Register Map datasheet
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

/*! Gyroscope Configuration Section 4.4 in Register Map datasheet
    Register  Bit 7   Bit 6   Bit 5     Bit 4  Bit 3     Bit 2   Bit 1   Bit 0
   |   1B   | XG_ST | YG_ST | ZG_ST | Full Scale Range |   -   |   -   |   -   |
   |        |       |       |       |   Select FS_SEL  |       |       |       |
   (Setting XG_ST, YG_ST or ZG_ST to 1 will perform a self test)

   Gyroscope Full Scale Range Selection:
    FS_SEL   Full Scale Range
   |   0   |   +/-  250 °/s   |
   |   1   |   +/-  500 °/s   |
   |   2   |   +/- 1000 °/s   |
   |   3   |   +/- 2000 °/s   |

   --> Note: FS_SEL/360 * 60sec/min = RotationValue in RPM (Rotations Per Minute)
*/
void CustomMPU6050Reader::gyroConfig() {
  switch (GY_RANGE) {
    case 0:
      writeTo(0x1B, 0x00);
      break;
    case 1:
      writeTo(0x1B, 0x08);
      break;
    case 2:
      writeTo(0x1B, 0x10);
      break;
    case 3:
      writeTo(0x1B, 0x18);
      break;
    default:
      writeTo(0x1B, 0x00);
  }
}

/*! Accelerometer Configuration Section 4.5 in the Register Map datasheet
    Register  Bit 7   Bit 6   Bit 5     Bit 4  Bit 3     Bit 2   Bit 1   Bit 0
   |   1C   | XA_ST | YA_ST | ZA_ST | Full Scale Range |   -   |   -   |   -   |
   |        |       |       |       |  Select AFS_SEL  |       |       |       |
   (Setting XA_ST, YA_ST or ZA_ST to 1 will perform a self test)

   Accelerometer Full Scale Range Selection:
    FS_SEL   Full Scale Range
   |   0   |     +/-   2g     |
   |   1   |     +/-   4g     |
   |   2   |     +/-   8g     |
   |   3   |     +/-  16g     |
*/
void CustomMPU6050Reader::accelConfig() {
  switch (AC_RANGE) {
    case 0:
      writeTo(0x1C, 0x00);
      break;
    case 1:
      writeTo(0x1C, 0x08);
      break;
    case 2:
      writeTo(0x1C, 0x10);
      break;
    case 3:
      writeTo(0x1C, 0x18);
      break;
    default:
      writeTo(0x1C, 0x00);
  }
}

/*!
  Computes offset values for the Gyroscope.
*/
void CustomMPU6050Reader::calibrateGyro() {
  int numSamples = 2000;
  float rateRoll, ratePitch, rateYaw;

  for (int i = 0; i < numSamples; i++) {
    recordGyroData(rateRoll, ratePitch, rateYaw);
    gyroXOffset += rateRoll;
    gyroYOffset += ratePitch;
    gyroZOffset += rateYaw;
    delay(1);
  }

  gyroXOffset /= numSamples;
  gyroYOffset /= numSamples;
  gyroZOffset /= numSamples;
}

/*!
  Compute offset values for the Accelerometer.
*/
void CustomMPU6050Reader::calibrateAccel() {
  int numSamples = 2000;
  float gForceX, gForceY, gForceZ;

  for (int i = 0; i < numSamples; i++) {
    recordAccelData(gForceX, gForceY, gForceZ);
    accXOffset += gForceX;
    accYOffset += gForceY;
    accZOffset += gForceZ;
    delay(1);
  }

  accXOffset /= numSamples;
  accYOffset /= numSamples;
  accZOffset = (accZOffset / numSamples) - 1;
}

/*! Gyroscope Measurements Section 4.19 in the Register Map datasheet
    Measurements are stored in registers 43 - 48.
    Note: each measurement is broken up into High and Low bytes.
*/
void CustomMPU6050Reader::recordGyroData(float& rateRoll, float& ratePitch, float& rateYaw) {
  uint16_t gyX, gyY, gyZ;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);                     // starting with register 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // request gyroscope measurements
  gyX = Wire.read()<<8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyY = Wire.read()<<8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyZ = Wire.read()<<8 | Wire.read();   // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  processGyroData(rateRoll, ratePitch, rateYaw, gyX, gyY, gyZ);
}

/*! Conversion of the raw Gyroscope measurements into exerted roll, pitch and yaw rates
    Section 4.19 Gyroscope Measurements
     
     FS_SEL   Full Scale Range    LSB Sensitivity
   |   0    |  +-  250°/s      |   131.0 LSB/°/s   |
   |   1    |  +-  500°/s      |    65.5 LSB/°/s   |
   |   2    |  +- 1000°/s      |    32.8 LSB/°/s   |
   |   3    |  +- 2000°/s      |    16.4 LSB/°/s   |

    --> Note: directional rotational force [°/s] = gyX/LSB Sensitivity
*/
void CustomMPU6050Reader::processGyroData(float& rateRoll, float& ratePitch, float& rateYaw,
                                          uint16_t gyX, uint16_t gyY, uint16_t gyZ) {
  float lsb_sens;

  switch (GY_RANGE) {
    case 0:
      lsb_sens = 131.0;
      break;
    case 1:
      lsb_sens = 65.5;
      break;
    case 2:
      lsb_sens = 32.8;
      break;
    case 3:
      lsb_sens = 16.4;
      break;
    default:
      lsb_sens = 131.0;
  }

  rateRoll = (float)(gyX) / lsb_sens;
  ratePitch = (float)(gyY) / lsb_sens;
  rateYaw = (float)(gyZ) / lsb_sens;

  if (calibrated) {
    rateRoll -= gyroXOffset;
    ratePitch -= gyroYOffset;
    rateYaw -= gyroZOffset;
  }
}

/*! Accelerometer Measurements Section 4.17 in the Register Map datasheet
    Measurements are stored in registers 3B - 40.
    Note: each measurement is broken up into High and Low bytes.
*/
void CustomMPU6050Reader::recordAccelData(float& gForceX, float& gForceY, float& gForceZ) {
  uint16_t acX, acY, acZ;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // request acceleration measurements
  acX = Wire.read()<<8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acY = Wire.read()<<8 | Wire.read();   // Ox3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acZ = Wire.read()<<8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  processAccelData(gForceX, gForceY, gForceZ, acX, acY, acZ);
}

/*! Conversion of the raw Accelerometer measurements into exerted g forces
    Section 4.17 Accelerometer Measurements
     
    AFS_SEL   Full Scale Range   LSB Sensitivity
   |   0    |      +-  2g      |   16384 LSB/g   |
   |   1    |      +-  4g      |    8192 LSB/g   |
   |   2    |      +-  8g      |    4096 LSB/g   |
   |   3    |      +- 16g      |    2048 LSB/g   |

    --> Note: directional g force = acX/LSB Sensitivity
*/
void CustomMPU6050Reader::processAccelData(float& gForceX, float& gForceY, float& gForceZ,
                                           uint16_t acX, uint16_t acY, uint16_t acZ) {
  float lsb_sens;

  switch (AC_RANGE) {
    case 0:
      lsb_sens = 16384.0;
      break;
    case 1:
      lsb_sens = 8192.0;
      break;
    case 2:
      lsb_sens = 4096.0;
      break;
    case 3:
      lsb_sens = 2048.0;
      break;
    default:
      lsb_sens = 16384.0;
  }

  gForceX = (float)(acX) / lsb_sens;
  gForceY = (float)(acY) / lsb_sens;
  gForceZ = (float)(acZ) / lsb_sens;

  if (calibrated) {
    gForceX -= accXOffset;
    gForceY -= accYOffset;
    gForceZ -= accZOffset;
  }
}

/*
  Default roll and pitch computations using only accelerometer.
*/
void CustomMPU6050Reader::computeAccRP(float& r, float& p, float gForceX, float gForceY, float gForceZ) {
  r = atan(gForceY / sqrt(pow(gForceX, 2) + pow(gForceZ, 2))) * 180/PI;
  p = atan(-gForceX / sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180/PI;
}

/*
  Low-pass filter roll and pitch computations using only accelerometer.
*/
void CustomMPU6050Reader::computeAccLpfRP(float& r, float& p, float gForceX, float gForceY, float gForceZ) {
  r = ALPHA_LPF * r + (1 - ALPHA_LPF) * atan(gForceY / sqrt(pow(gForceX, 2) + pow(gForceZ, 2))) * 180/PI;
  p = ALPHA_LPF * p + (1 - ALPHA_LPF) * atan(-gForceX / sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180/PI;
}

/*
  Default roll, pitch and yaw (rpy) computations using only gyroscope.
*/
void CustomMPU6050Reader::computeGyroRPY(float& r, float& p, float& y,
                                         float rateRoll, float ratePitch, float rateYaw,
                                         float& curr_time, float& delta_time, float& prev_time) {
  curr_time = millis();
  delta_time = (curr_time - prev_time) / 1000.0;
  prev_time = curr_time;

  r += rateRoll * delta_time;
  p += ratePitch * delta_time;
  y += rateYaw * delta_time;
}

/*
  Complementary filter roll, pitch and yaw computations using sensor fusion
  of gyroscope and accelerometer measurements.
*/
void CustomMPU6050Reader::computeComplRPY(float& r, float& p, float& y,
                                          float gForceX, float gForceY, float gForceZ,
                                          float rateRoll, float ratePitch, float rateYaw,
                                          float& curr_time, float& delta_time, float& prev_time) {
  float accRoll, accPitch;
  computeAccRP(accRoll, accPitch, gForceX, gForceY, gForceZ);
  curr_time = millis();
  delta_time = (curr_time - prev_time) / 1000.0;
  prev_time = curr_time;

  r = ALPHA_COMPL_FILTER * (r + rateRoll * delta_time);
  r += (1 - ALPHA_COMPL_FILTER) * accRoll;
  p = ALPHA_COMPL_FILTER * (p + ratePitch * delta_time);
  p += (1 - ALPHA_COMPL_FILTER) * accPitch;
  y += rateYaw * delta_time;
}
