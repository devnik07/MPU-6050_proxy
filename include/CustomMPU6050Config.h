#pragma once

#include "IConfig.h"
#include "ArduinoEEPROMManager.h"
#include "CustomMPU6050Reader.h"

class CustomMPU6050Config : public IConfig {
    private:
        ArduinoEEPROMManager eepromManager;
        CustomMPU6050Reader reader;

    public:
        CustomMPU6050Config(const ComputationOption compOpt);
        IIMUReader& getReader() override;
};