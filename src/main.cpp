#include <Arduino.h>

#define USE_CUSTOM_READER

#ifdef USE_CUSTOM_READER
  #include "CustomMPU6050Reader.h"
#else
  #include "MPU6050Reader.h"
#endif

void printRPY();

const int LOOP_DELAY = 100;

#ifdef USE_CUSTOM_READER
  const ComputationOption compOpt = ComputationOption::COMPL_RPY;
  CustomMPU6050Reader reader(compOpt);
#else
  MPU6050 mpu;
  MPU6050Reader reader(mpu);
#endif

float roll = 0, pitch = 0, yaw = 0;

void setup() {
  Serial.begin(115200); // starts serial monitor
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