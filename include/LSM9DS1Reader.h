#pragma once

#include "IIMUReader.h"

#include <LSM9DS1.h>
#include <MadgwickAHRS.h>

class LSM9DS1Reader : public IIMUReader {
    private:
        Madgwick& filter;
        float gyroXOffset, gyroYOffset, gyroZOffset;
        float accXOffset, accYOffset, accZOffset;

    public:
        LSM9DS1Reader(Madgwick& filter);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;
        void getRotationQuaternion(float& w, float& x, float& y, float& z) override;
        void calibrate() override;
        bool isCalibrated() override;
        void resetCalibrationFlag() override;

    private:
        void calibrateGyro();
        void calibrateAccel();
        void loadCalibrationOffsets();
};