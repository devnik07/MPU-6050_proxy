#pragma once

#include "IMUReader.h"
#include <stdlib.h>

const int MPU_ADDR = 0x68;              // I2C address of the MPU-6050
const int DLPF_CFG = 0;                 // Digital Low Pass Filter Configuration (values: 0-6)           
const int GY_RANGE = 0;                 // Full Scale Range of Gyroscope Outputs (values: 0-3)
const int AC_RANGE = 0;                 // Full Scale Range of Accelerometer Outputs (values: 0-3)
const float ALPHA_LPF = 0.8;
const float ALPHA_COMPL_FILTER = 0.9;

/* Defines in which way roll, pitch and yaw are to be computed */
enum class ComputationOption {
    ACC_RP, /*!< Only use accelerometer measurements to compute roll/pitch */
    ACC_RP_LPF, /*!< Use low pass filter to compute roll/pitch using accelerometer */
    GYRO_RPY, /*!< Only use gyroscope measurements to compute roll/pitch/yaw */
    COMPL_RPY /*!< Use sensor fusion to compute roll/pitch/yaw */
};

class CustomMPU6050Reader : public IMUReader {
    private:
        float gyroXOffset, gyroYOffset, gyroZOffset;
        float accXOffset, accYOffset, accZOffset;
        bool calibrated;
        const ComputationOption computationOption;

    public:
        CustomMPU6050Reader(ComputationOption compOpt);
        void init() override;
        void getRollPitchYaw(float& r, float& p, float& y) override;

    private:
        void writeTo(uint8_t address, uint8_t value);
        void wakeUp();
        void setLowPassFilter();
        void gyroConfig();
        void accelConfig();
        void calibrateGyro();
        void calibrateAccel();
        void recordGyroData(float& rateRoll, float& ratePitch, float& rateYaw);
        void recordAccelData(float& gForceX, float& gForceY, float& gForceZ);
        void processGyroData(float& rateRoll, float& ratePitch, float& rateYaw,
                             uint16_t gyX, uint16_t gyY, uint16_t gyZ);
        void processAccelData(float& gForceX, float& gForceY, float& gForceZ,
                              uint16_t acX, uint16_t acY, uint16_t acZ);
        void computeAccRP(float& r, float& p,
                          float gForceX, float gForceY, float gForceZ);
        void computeAccLpfRP(float& r, float& p,
                             float gForceX, float gForceY, float gForceZ);
        void computeGyroRPY(float& r, float& p, float& y,
                            float rateRoll, float ratePitch, float rateYaw,
                            float& curr_time, float& delta_time, float& prev_time);  
        void computeComplRPY(float& r, float& p, float& y,
                            float gForceX, float gForceY, float gForceZ,
                            float rateRoll, float ratePitch, float rateYaw,
                            float& curr_time, float& delta_time, float& prev_time);  
};
