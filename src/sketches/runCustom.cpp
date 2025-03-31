#include <Arduino.h>

#include "IConfig.h"
#include "CustomMPU6050Config.h"
#include "JoystickReader.h"

void printRPY();
void printJoystickInputs();

const int LOOP_DELAY = 100;

const ComputationOption compOpt = ComputationOption::COMPL_RPY;
CustomMPU6050Config mpuConfig(compOpt);
IConfig& config = mpuConfig;
IIMUReader& reader = config.getReader();

float roll = 0, pitch = 0, yaw = 0;
int joystickX, joystickY;
bool switchOn = false;

void setup() {
    Serial.begin(115200);     // starts serial communication
    while (!Serial);

    reader.init();
    JoystickReader::init();
}

void loop() {
  reader.getRollPitchYaw(roll, pitch, yaw);
  printRPY();

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
  Serial.print(yaw);
  Serial.print(",");
}

void printJoystickInputs() {
  Serial.print(joystickX);
  Serial.print(",");
  Serial.print(joystickY);
  Serial.print(",");
  Serial.println(switchOn ? 1 : 0);
}