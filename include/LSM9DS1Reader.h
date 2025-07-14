#pragma once

#include "IIMUReader.h"
#include "IStorageManager.h"

#include <LSM9DS1.h>
#include <MadgwickAHRS.h>

class LSM9DS1Reader : public IIMUReader {
    private:
        IStorageManager<float>& flashManager;
        Madgwick& filter;
        float gyroXOffset, gyroYOffset, gyroZOffset;
        float accXOffset, accYOffset, accZOffset;
        float magXOffset, magYOffset, magZOffset;
        float yawReference;

    public:
        LSM9DS1Reader(IStorageManager<float>& iflashManager, Madgwick& filter);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;
        void getRotationQuaternion(float& w, float& x, float& y, float& z) override;
        void calibrate() override;
        bool isCalibrated() override;
        void resetCalibrationFlag() override;

    private:
        void calibrateGyro();
        void calibrateAccel();
        void calibrateMag();
        void loadCalibrationOffsets();
        void setupFilter();
};