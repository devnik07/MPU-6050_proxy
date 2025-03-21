#include <Arduino.h>

#include "IConfig.h"
#include "CustomMPU6050Config.h"

void printRPY();

const int LOOP_DELAY = 100;

const ComputationOption compOpt = ComputationOption::COMPL_RPY;
CustomMPU6050Config mpuConfig(compOpt);
IConfig& config = mpuConfig;
IIMUReader& reader = config.getReader();

float roll = 0, pitch = 0, yaw = 0;

void setup() {
    Serial.begin(115200);     // starts serial communication
    while (!Serial);

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