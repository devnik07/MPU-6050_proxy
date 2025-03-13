#pragma once

#include "IConfig.h"
#include "ArduinoEEPROMManager.h"
#include "MPU6050Reader.h"

class MPU6050Config : public IConfig {
    private:
        ArduinoEEPROMManager eepromManager;
        MPU6050Reader reader;

    public:
        MPU6050Config(MPU6050& mpu);
        IIMUReader& getReader() override;
};