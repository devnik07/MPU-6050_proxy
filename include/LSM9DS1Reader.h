#pragma once

#include "IIMUReader.h"
#include "IStorageManager.h"

#include <LSM9DS1.h>
#include <MadgwickAHRS.h>

// Soft iron distortions compensation matrix
const float S_MAG[3][3] = {
  {1.332299, 0.044408, -0.002981},
  {0.044408, 1.430956, -0.040132},
  {-0.002981, -0.040132, 1.296526}
};

// Hard iron distortions compensation bias
const float H_MAG[3] = {
  -28.599320, 36.203492, 38.307712
};

const float S_ACC[3][3] = {
  {1.002586, -0.001156, 0.002170},
  {-0.001156, 1.002056, -0.000748},
  {0.002170, -0.000748, 1.001003}
};

const float H_ACC[3] = {
  -0.031949, -0.003886, -0.006204
};

const int SETUP_TIME = 15000;

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
        void updateFilter();
        void scaleIMU(float& rateRoll, float& ratePitch, float& rateYaw,
                      float& gForceX, float& gForceY, float& gForceZ,
                      float& mX, float& mY, float& mZ);
};