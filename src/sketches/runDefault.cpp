#include <Arduino.h>

#include "IConfig.h"
#include "MPU6050Config.h"
#include "JoystickReader.h"

void printRPY();
void printRotationQuaternion();
void printJoystickInputs();

const int LOOP_DELAY = 100;

MPU6050 mpu;
MPU6050Config mpuConfig(mpu);
IConfig& config = mpuConfig;
IIMUReader& reader = config.getReader();

float roll = 0, pitch = 0, yaw = 0;
float w, x, y, z;
int joystickX, joystickY;
bool switchOn = false;

void setup() {
  Serial.begin(115200); // starts serial communication
  while (!Serial);

  reader.init();
  JoystickReader::init();
}

void loop() {
  //reader.getRollPitchYaw(roll, pitch, yaw);
  //printRPY();
  reader.getRotationQuaternion(w, x, y, z);
  printRotationQuaternion();

  JoystickReader::getInputs(joystickX, joystickY, switchOn);
  printJoystickInputs();

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

void printRotationQuaternion() {
  Serial.print(w);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");
}

void printJoystickInputs() {
  Serial.print(joystickX);
  Serial.print(",");
  Serial.print(joystickY);
  Serial.print(",");
  Serial.println(switchOn);
}