#pragma once

#include "IIMUReader.h"
#include "IStorageManager.h"

#include "MPU6050_6Axis_MotionApps20.h"

class MPU6050Reader : public IIMUReader {
    private:
        IStorageManager<int16_t>& eepromManager;
        MPU6050& mpu;
        uint8_t FIFOBuffer[64];

    public:
        MPU6050Reader(IStorageManager<int16_t>& ieepromManager, MPU6050& mpu6050);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;
        void getRotationQuaternion(float& w, float& x, float& y, float& z) override;
        void calibrate() override;
        bool isCalibrated() override;
        void resetCalibrationFlag() override;

    private:
        void loadCalibrationOffsets();
        uint8_t setupMPU();
        void setStartingOffsets();
    
};
