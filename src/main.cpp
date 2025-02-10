#include <Arduino.h>
#include <Wire.h>

const int MPU_addr = 0x68;              // I2C address of the MPU-6050
const int LOOP_DELAY = 333;
const int DLPF_CFG = 0;                 // Digital Low Pass Filter Configuration (values: 0-6)           
const int GY_RANGE = 0;                 // Full Scale Range of Gyroscope Outputs (values: 0-3)
const int AC_RANGE = 0;                 // Full Scale Range of Accelerometer Outputs (values: 0-3)
const float ALPHA_LPF = 0.8;

int16_t acX, acY, acZ;
float gForceX, gForceY, gForceZ;

int16_t gyX, gyY, gyZ;
float rateRoll, ratePitch, rateYaw;

float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float accXOffset = 0, accYOffset = 0, accZOffset = 0;

bool calibrated = false;

float roll_acc = 0, pitch_acc = 0;
float roll_gyro = 0, pitch_gyro = 0, yaw_gyro = 0;

float prev_time = 0, curr_time = 0, delta_time = 0;

void write_to(byte address, byte value);
void wake_up();
void set_low_pass_filter();
void gyro_config();
void accel_config();
void record_accel_data();
void record_gyro_data();
void process_accel_data();
void process_gyro_data();
void acc_rp();
void acc_rp_lpf();
void gyro_rpy();
void calibrate_gyro();
void calibrate_acc();
void print_measurements();
void plot_gyro_rates();

void print_acc_rp();
void print_gyro_rpy();


void setup() {
  Serial.begin(9600); // starts serial monitor

  Wire.begin();
  wake_up();

  set_low_pass_filter();
  gyro_config();
  accel_config();

  calibrate_gyro();
  calibrate_acc();
  calibrated = true;
}

void loop() {
  record_accel_data();
  record_gyro_data();

  // acc_rp();
  // gyro_rpy();
  acc_rp_lpf();

  //print_measurements();
  //plot_gyro_rates();
  //print_acc_rp(); 
  print_gyro_rpy();

  delay(LOOP_DELAY);
}

// TODO: write documentation for this method
void write_to(byte address, byte value) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

/* Power Management 1 Section 4.28 in Register Map datasheet
    Wakes up the MPU-6050 from sleep mode
*/
void wake_up() {
  write_to(0x6B, 0x00);
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
void set_low_pass_filter() {
  switch (DLPF_CFG) {
    case 0:
      write_to(0x1A, 0x00);
      break;
    case 1:
      write_to(0x1A, 0x01);
      break;
    case 2:
      write_to(0x1A, 0x02);
      break;
    case 3:
      write_to(0x1A, 0x03);
      break;
    case 4:
      write_to(0x1A, 0x04);
      break;
    case 5:
      write_to(0x1A, 0x05);
      break;
    case 6:
      write_to(0x1A, 0x06);
      break;
    default:
      write_to(0x1A, 0x00);
  }
}

/* Gyroscope Configuration Section 4.4 in Register Map datasheet
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
void gyro_config() {
  switch (GY_RANGE) {
    case 0:
      write_to(0x1B, 0x00);
      break;
    case 1:
      write_to(0x1B, 0x08);
      break;
    case 2:
      write_to(0x1B, 0x10);
      break;
    case 3:
      write_to(0x1B, 0x18);
      break;
    default:
      write_to(0x1B, 0x00);
  }
}

/* Accelerometer Configuration Section 4.5 in the Register Map datasheet
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
void accel_config() {
  switch (AC_RANGE) {
    case 0:
      write_to(0x1C, 0x00);
      break;
    case 1:
      write_to(0x1C, 0x08);
      break;
    case 2:
      write_to(0x1C, 0x10);
      break;
    case 3:
      write_to(0x1C, 0x18);
      break;
    default:
      write_to(0x1C, 0x00);
  }
}

/* Accelerometer Measurements Section 4.17 in the Register Map datasheet
    Measurements are stored in registers 3B - 40.
    Note: each measurement is broken up into High and Low bytes.
*/
void record_accel_data() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);  // request acceleration measurements
  acX = Wire.read()<<8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acY = Wire.read()<<8 | Wire.read();   // Ox3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acZ = Wire.read()<<8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  process_accel_data();
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
void process_accel_data() {
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

/* Gyroscope Measurements Section 4.19 in the Register Map datasheet
    Measurements are stored in registers 43 - 48.
    Note: each measurement is broken up into High and Low bytes.
*/
void record_gyro_data() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);                     // starting with register 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);  // request gyroscope measurements
  gyX = Wire.read()<<8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyY = Wire.read()<<8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyZ = Wire.read()<<8 | Wire.read();   // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  process_gyro_data();
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
void process_gyro_data() {
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

/*
  Default roll and pitch computations using only accelerometer.
*/
void acc_rp() {
  roll_acc = atan(gForceY / sqrt(pow(gForceX, 2) + pow(gForceZ, 2))) * 180/PI;
  pitch_acc = atan(-gForceX / sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180/PI;
}

/*
  Low-pass filter roll and pitch computations using only accelerometer.
*/
void acc_rp_lpf() {
  roll_acc = ALPHA_LPF * roll_acc + (1 - ALPHA_LPF) * atan(gForceY / sqrt(pow(gForceX, 2) + pow(gForceZ, 2))) * 180/PI;
  pitch_acc = ALPHA_LPF * pitch_acc + (1 - ALPHA_LPF) * atan(-gForceX / sqrt(pow(gForceY, 2) + pow(gForceZ, 2))) * 180/PI;
}

/*
  Default roll, pitch and yaw (rpy) computations using only gyroscope.
*/
void gyro_rpy() {
  curr_time = millis();
  delta_time = (curr_time - prev_time) / 1000.0;
  prev_time = curr_time;

  roll_gyro = roll_gyro + rateRoll * delta_time;
  pitch_gyro = pitch_gyro + ratePitch * delta_time;
  yaw_gyro = yaw_gyro + rateYaw * delta_time;
}

/*
  Computes offset values for the Gyroscope.
*/
void calibrate_gyro() {
  Serial.println("Calibrating Gyroscope ...");

  int numSamples = 2000;

  for (int i = 0; i < numSamples; i++) {
    record_gyro_data();
    gyroXOffset += rateRoll;
    gyroYOffset += ratePitch;
    gyroZOffset += rateYaw;
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
void calibrate_acc() {
  Serial.println("Calibrating Accelerometer ...");

  int numSamples = 2000;

  for (int i = 0; i < numSamples; i++) {
    record_accel_data();
    accXOffset += gForceX;
    accYOffset += gForceY;
    accZOffset += gForceZ;
    delay(1);
  }

  accXOffset /= numSamples;
  accYOffset /= numSamples;
  accZOffset = (accZOffset / numSamples) - 1;

  Serial.println("Accelerometer Calibration completed.");
}

void print_measurements() {
  Serial.print("Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
  Serial.print("\t");
  Serial.print("Gyro (deg/s)");
  Serial.print(" X=");
  Serial.print(rateRoll);
  Serial.print(" Y=");
  Serial.print(ratePitch);
  Serial.print(" Z=");
  Serial.print(rateYaw);
  Serial.print("\n");
}

void plot_gyro_rates() {
  Serial.print("RollRate:");
  Serial.print(rateRoll);
  Serial.print(",");
  Serial.print("PitchRate:");
  Serial.print(ratePitch);
  Serial.print(",");
  Serial.print("YawRate:");
  Serial.println(rateYaw);
}

void print_acc_rp() {
  Serial.print("Roll:");
  Serial.print(roll_acc);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.println(pitch_acc);
}

void print_gyro_rpy() {
  Serial.print("Roll:");
  Serial.print(roll_gyro);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(pitch_gyro);
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.println(yaw_gyro);
}