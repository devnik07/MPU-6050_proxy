#include <Arduino.h>

#include "IConfig.h"
#include "LSM9DS1Config.h"
#include "JoystickReader.h"

void printRPY();
void printRotationQuaternion();
void printJoystickInputs();

const int SAMPLE_RATE = 119;
const int LOOP_DELAY = 16;

Madgwick filter;
LSM9DS1Config imuConfig(filter);
IConfig& config = imuConfig;
IIMUReader& reader = config.getReader();

float roll = 0, pitch = 0, yaw = 0;
float w, x, y, z;
int joystickX, joystickY;
bool switchOn = false;

unsigned long microsPerReading, microsPrevious;
unsigned long microsNow;
unsigned long prevPrint;

void setup() {
    Serial.begin(9600); // starts serial communication
    while (!Serial);

    reader.init();
    JoystickReader::init();

    microsPerReading = 1000000 / SAMPLE_RATE;
    microsPrevious = micros();
    prevPrint = millis();
}

void loop() {
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
      
      reader.getRollPitchYaw(roll, pitch, yaw);
      //reader.getRotationQuaternion(w, x, y, z);
      JoystickReader::getInputs(joystickX, joystickY, switchOn);

      microsPrevious += microsPerReading;

      if (millis() - prevPrint >= LOOP_DELAY) {
        printRPY();
        //printRotationQuaternion();
        printJoystickInputs();

        if (switchOn) {
          reader.calibrate();
        }
        prevPrint = millis();
      }
    }
}

void printRPY() {
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
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