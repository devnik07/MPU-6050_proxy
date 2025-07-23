#pragma once

#include "IIMUReader.h"
#include "IStorageManager.h"

#include <LSM9DS1.h>
#include <MadgwickAHRS.h>

// Soft iron distortions compensation matrix
const float A_SI[3][3] = {
  {1.338425, 0.056809, 0.006194},
  {0.056809, 1.223389, -0.009664},
  {0.006194, -0.009664, 1.260731}
};

// Hard iron distortions compensation bias
const float B_HI[3] = {
  -21.378609, 30.574078, 37.794415
};

const float A_ACC[3][3] = {
  {1.002586, -0.001156, 0.002170},
  {-0.001156, 1.002056, -0.000748},
  {0.002170, -0.000748, 1.001003}
};

const float B_ACC[3] = {
  -0.031949, -0.003886, -0.006204
};

const int SETUP_TIME = 7500;

class LSM9DS1Reader : public IIMUReader {
    private:
        IStorageManager<float>& flashManager;
        Madgwick& filter;
        float gyroXOffset, gyroYOffset, gyroZOffset;
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
        void loadCalibrationOffsets();
        void setupFilter();
        void updateFilter();
        void scaleIMU(float& rateRoll, float& ratePitch, float& rateYaw,
                      float& gForceX, float& gForceY, float& gForceZ,
                      float& mX, float& mY, float& mZ);
};