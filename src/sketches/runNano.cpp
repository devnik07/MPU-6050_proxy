#include <Arduino.h>
#include <ArduinoBLE.h>

#include "IConfig.h"
#include "LSM9DS1Config.h"
#include "JoystickReader.h"

#define BLE_UUID_SERVICE      "075779B4-B326-4EA8-A655-E8B9BE29C69F"
#define BLE_UUID_ORIENTATION  "44FFA62C-0256-44F3-8909-C1FFAC456B0F"
#define BLE_UUID_JOYSTICK     "CED2881A-F7F4-4614-8918-03C932409FDD"
#define BLE_UUID_CALIBRATION  "9574E0FE-622C-480B-88B2-16D7EE083784"

void printRPY();
void printRotationQuaternion();
void printJoystickInputs();

union orientation_data
{
  struct __attribute__( ( packed ))
  {
    float quaternion[4];
  };
  uint8_t bytes[4 * sizeof(float)];
};
union orientation_data orientationData;

union joystick_data
{
  struct __attribute__( ( packed ))
  {
    int joystickInput[2];
  };
  uint8_t bytes[2 * sizeof(int)];
};
union joystick_data joystickData;

const int SAMPLE_RATE = 119;

Madgwick filter;
LSM9DS1Config imuConfig(filter);
IConfig& config = imuConfig;
IIMUReader& reader = config.getReader();

BLEService caxeService(BLE_UUID_SERVICE);
BLECharacteristic orientationCharacteristic(BLE_UUID_ORIENTATION, BLERead | BLENotify, sizeof orientationData.bytes);
BLECharacteristic joystickCharacteristic(BLE_UUID_JOYSTICK, BLERead | BLENotify, sizeof joystickData.bytes);
BLEBoolCharacteristic calibrationCharacteristic(BLE_UUID_CALIBRATION, BLERead | BLENotify);

float roll = 0, pitch = 0, yaw = 0;
float w, x, y, z;
int joystickX, joystickY;
bool switchOn = false;

const int LOOP_DELAY = 100;
unsigned long microsPerReading, microsPrevious;
unsigned long microsNow;
unsigned long prevPrint;

void setup() {
    Serial.begin(9600); // starts serial communication
    while (!Serial);

    reader.init();
    JoystickReader::init();

    if (!BLE.begin()) {
      Serial.println("Starting Bluetooth® Low Energy module failed.");
      while (true);   // stop execution
    }

    BLE.setLocalName("CAXE");
    BLE.setAdvertisedService(caxeService);

    caxeService.addCharacteristic(orientationCharacteristic);
    caxeService.addCharacteristic(joystickCharacteristic);
    caxeService.addCharacteristic(calibrationCharacteristic);

    BLE.addService(caxeService);
    calibrationCharacteristic.setValue(false);

    BLE.advertise();

    microsPerReading = 1000000 / SAMPLE_RATE;
    microsPrevious = micros();
    prevPrint = millis();
}

void loop() {
    BLEDevice central = BLE.central();
    if (central) {
      Serial.print("Connected to central: ");
      Serial.println(central.address());

      while (central.connected()) {
        microsNow = micros();
        if (microsNow - microsPrevious >= microsPerReading) {
          reader.getRotationQuaternion(w, x, y, z);
          JoystickReader::getInputs(joystickX, joystickY, switchOn);

          microsPrevious += microsPerReading;
        }

        if (millis() - prevPrint >= LOOP_DELAY) {
          orientationData.quaternion[0] = w;
          orientationData.quaternion[1] = x;
          orientationData.quaternion[2] = y;
          orientationData.quaternion[3] = z;
          orientationCharacteristic.writeValue(orientationData.bytes, sizeof orientationData.bytes);

          joystickData.joystickInput[0] = joystickX;
          joystickData.joystickInput[1] = joystickY;
          joystickCharacteristic.writeValue(joystickData.bytes, sizeof joystickData.bytes);

          calibrationCharacteristic.writeValue(switchOn);

          printRotationQuaternion();
          printJoystickInputs();

          if (switchOn) {
            reader.calibrate();
          }
          prevPrint = millis();
        }
      }

      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
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