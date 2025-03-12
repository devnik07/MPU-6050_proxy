#pragma once

#include "IIMUReader.h"
#include "IeepromMPU.h"

#include "MPU6050_6Axis_MotionApps20.h"

class MPU6050Reader : public IIMUReader {
    private:
        IeepromMPU& eepromManager;
        MPU6050& mpu;
        uint8_t FIFOBuffer[64];

    public:
        MPU6050Reader(IeepromMPU& ieepromManager, MPU6050& mpu6050);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;
        void calibrate() override;
        bool isCalibrated() override;
        void resetCalibrationFlag() override;

    private:
        void loadCalibrationOffsets();
        uint8_t setupMPU();
        void setStartingOffsets();
    
};
