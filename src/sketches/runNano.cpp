#include <Arduino.h>

#include "IConfig.h"
#include "LSM9DS1Config.h"
#include "JoystickReader.h"

void printRPY();
void printRotationQuaternion();
void printJoystickInputs();

const int LOOP_DELAY = 100;

Madgwick filter;
LSM9DS1Config imuConfig(filter);
IConfig& config = imuConfig;
IIMUReader& reader = config.getReader();

float roll = 0, pitch = 0, yaw = 0;
float w, x, y, z;
int joystickX, joystickY;
bool switchOn = false;

void setup() {
    Serial.begin(9600); // starts serial communication
    while (!Serial);

    reader.init();
    JoystickReader::init();
}

void loop() {
    reader.getRotationQuaternion(w, x, y, z);
    printRotationQuaternion();

    JoystickReader::getInputs(joystickX, joystickY, switchOn);
    printJoystickInputs();

    delay(LOOP_DELAY);
}

void printRPY() {
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
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
  Serial.println(switchOn ? 1 : 0);
}