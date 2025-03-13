#include <Arduino.h>

#include "IConfig.h"
#include "CustomMPU6050Config.h"

const ComputationOption compOpt = ComputationOption::COMPL_RPY;
CustomMPU6050Config mpuConfig(compOpt);
IConfig& config = mpuConfig;
IIMUReader& reader = config.getReader();

void setup() {
    Serial.begin(9600);     // starts serial communication
    while (!Serial);

    reader.calibrate();
}

void loop() {}