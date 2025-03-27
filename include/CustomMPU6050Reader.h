#pragma once

#include "IIMUReader.h"
#include "IeepromMPU.h"
#include <stdint.h>

const int MPU_ADDR = 0x68;              // I2C address of the MPU-6050
const int DLPF_CFG = 0;                 // Digital Low Pass Filter Configuration (values: 0-6)           
const int GY_RANGE = 0;                 // Full Scale Range of Gyroscope Outputs (values: 0-3)
const int AC_RANGE = 0;                 // Full Scale Range of Accelerometer Outputs (values: 0-3)
const float ALPHA_LPF = 0.8;
const float ALPHA_COMPL_FILTER = 0.95;

/* Defines in which way roll, pitch and yaw are to be computed */
enum class ComputationOption {
    ACC_RP, /*!< Only use accelerometer measurements to compute roll/pitch */
    ACC_RP_LPF, /*!< Use low pass filter to compute roll/pitch using accelerometer */
    GYRO_RPY, /*!< Only use gyroscope measurements to compute roll/pitch/yaw */
    COMPL_RPY /*!< Use sensor fusion to compute roll/pitch/yaw */
};

class CustomMPU6050Reader : public IIMUReader {
    private:
        IeepromMPU& eepromManager;
        float gyroLSBSens, accLSBSens;
        float gyroXOffset, gyroYOffset, gyroZOffset;
        float accXOffset, accYOffset, accZOffset;
        float prevTime;
        const ComputationOption computationOption;

    public:
        CustomMPU6050Reader(IeepromMPU& ieepromManager, ComputationOption compOpt);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;
        void getRotationQuaternion(float& w, float& x, float& y, float& z) override;
        void calibrate() override;
        bool isCalibrated() override;
        void resetCalibrationFlag() override;

    private:
        void loadCalibrationOffsets();
        void writeTo(uint8_t address, uint8_t value);
        void setupMPU();
        void wakeUp();
        void setLowPassFilter();
        void gyroConfig();
        void accelConfig();
        void calibrateGyro();
        void calibrateAccel();
        void readGyroData(int16_t& gyX, int16_t& gyY, int16_t& gyZ);
        void recordGyroRates(float& rateRoll, float& ratePitch, float& rateYaw);
        void processGyroData(float& rateRoll, float& ratePitch, float& rateYaw,
                             int16_t gyX, int16_t gyY, int16_t gyZ);
        void readAccelData(int16_t& acX, int16_t& acY, int16_t& acZ);
        void recordAccelRates(float& gForceX, float& gForceY, float& gForceZ);
        void processAccelData(float& gForceX, float& gForceY, float& gForceZ,
                              int16_t acX, int16_t acY, int16_t acZ);
        void computeAccRP(float& r, float& p,
                          float gForceX, float gForceY, float gForceZ);
        void computeAccLpfRP(float& r, float& p,
                             float gForceX, float gForceY, float gForceZ);
        void computeGyroRPY(float& r, float& p, float& y,
                            float rateRoll, float ratePitch, float rateYaw);  
        void computeComplRPY(float& r, float& p, float& y,
                            float gForceX, float gForceY, float gForceZ,
                            float rateRoll, float ratePitch, float rateYaw);  
};
