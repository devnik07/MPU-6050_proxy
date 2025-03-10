#pragma once

#include "IIMUReader.h"

#include "MPU6050_6Axis_MotionApps20.h"

class MPU6050Reader : public IIMUReader {
    private:
        MPU6050& mpu;
        uint8_t FIFOBuffer[64];

    public:
        MPU6050Reader(MPU6050& mpu);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;

    private:
        void setStartingOffsets();
    
};
