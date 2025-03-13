#include <Arduino.h>

#include "IConfig.h"
#include "MPU6050Config.h"

MPU6050 mpu;
MPU6050Config mpuConfig(mpu);
IConfig& config = mpuConfig;
IIMUReader& reader = config.getReader();

void setup() {
    Serial.begin(9600);     // starts serial monitor
    while (!Serial);

    reader.resetCalibrationFlag();
}

void loop() {}