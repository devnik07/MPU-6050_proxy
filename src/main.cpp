#include <Arduino.h>

#include "IConfig.h"

// #define USE_CUSTOM_READER

#ifdef USE_CUSTOM_READER
  #include "CustomMPU6050Config.h"
#else
  #include "MPU6050Config.h"
#endif

void printRPY();

const int LOOP_DELAY = 100;

#ifdef USE_CUSTOM_READER
  const ComputationOption compOpt = ComputationOption::COMPL_RPY;
  CustomMPU6050Config mpuConfig(compOpt);
#else
  MPU6050 mpu;
  MPU6050Config mpuConfig(mpu);
#endif

IConfig& config = mpuConfig;
IIMUReader& reader = config.getReader();

float roll = 0, pitch = 0, yaw = 0;

void setup() {
  Serial.begin(9600); // starts serial monitor
  while (!Serial);

  //reader.resetCalibrationFlag();
  //reader.calibrate();
  reader.init();
}

void loop() {
  reader.getRollPitchYaw(roll, pitch, yaw);
  printRPY();
  delay(LOOP_DELAY);
}

void printRPY() {
  //Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(",");
  //Serial.print("Pitch:");
  Serial.print(pitch);
  Serial.print(",");
  //Serial.print("Yaw:");
  Serial.println(yaw);
}