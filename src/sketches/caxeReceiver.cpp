#include <Arduino.h>
#include <ArduinoBLE.h>

#define BLE_UUID_SERVICE              "075779B4-B326-4EA8-A655-E8B9BE29C69F"
#define BLE_UUID_ORIENTATION  "44FFA62C-0256-44F3-8909-C1FFAC456B0F"
#define BLE_UUID_JOYSTICK     "CED2881A-F7F4-4614-8918-03C932409FDD"
#define BLE_UUID_CALIBRATION  "9574E0FE-622C-480B-88B2-16D7EE083784"

void readCAXE(BLEDevice caxeController);

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

int switchOn = false;

void setup() {
    Serial.begin(9600); // starts serial communication
    while (!Serial);

    BLE.begin();
    BLE.scanForUuid(BLE_UUID_SERVICE);
}

void loop() {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
        if (peripheral.localName() != "CAXE") {
            return;
        }
        BLE.stopScan();
        readCAXE(peripheral);
    }
    BLE.scanForUuid(BLE_UUID_SERVICE);
}

void readCAXE(BLEDevice caxeController) {
    Serial.println("Connecting ...");

    if (caxeController.connect()) {
        Serial.println("Connected.");
    } else {
        Serial.println("Failed to connect!");
        return;
    }

    Serial.println("Discovering attributes ...");
    if (caxeController.discoverAttributes()) {
        Serial.println("Attributes discovered.");
    } else {
        Serial.println("Attribute discovery failed.");
        caxeController.disconnect();
        return;
    }

    BLECharacteristic orientationCharacteristic = caxeController.characteristic(BLE_UUID_ORIENTATION);
    BLECharacteristic joystickCharacteristic = caxeController.characteristic(BLE_UUID_JOYSTICK);
    BLECharacteristic calibrationCharacteristic = caxeController.characteristic(BLE_UUID_CALIBRATION);

    if (!orientationCharacteristic | !joystickCharacteristic | !calibrationCharacteristic) {
        caxeController.disconnect();
        return;
    }
    Serial.println("BLE Characteristics found.");

    if (!orientationCharacteristic.canSubscribe() | !joystickCharacteristic.canSubscribe() |
        !calibrationCharacteristic.canSubscribe()) {
        caxeController.disconnect();
        return;
    }
    if (!orientationCharacteristic.subscribe() | !joystickCharacteristic.subscribe() |
        !calibrationCharacteristic.subscribe()) {
        caxeController.disconnect();
        return;
    }
    Serial.println("BLE Characteristics subscribed.");

    while (caxeController.connected()) {
        orientationCharacteristic.readValue(orientationData.bytes, sizeof orientationData.bytes);
        joystickCharacteristic.readValue(joystickData.bytes, sizeof joystickData.bytes);
        calibrationCharacteristic.readValue(&switchOn, 1);

        for (int i = 0; i < 4; i++) {
            Serial.print(orientationData.quaternion[i]);
            Serial.print(",");
        }
        Serial.print(joystickData.joystickInput[0]);
        Serial.print(",");
        Serial.print(joystickData.joystickInput[1]);
        Serial.print(",");
        Serial.println(switchOn ? 1 : 0);
    }

    caxeController.disconnect();
    Serial.println("Disconnected.");
}


