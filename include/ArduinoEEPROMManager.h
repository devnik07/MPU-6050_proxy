#pragma once

#include "IeepromMPU.h"

class ArduinoEEPROMManager : public IeepromMPU {
    private:
        const int16_t ADDR_CALIBRATION_FLAG = 0;

        enum OffsetAddr {
            ADDR_X_GYRO_OFFSET = 1,
            ADDR_Y_GYRO_OFFSET = 3,
            ADDR_Z_GYRO_OFFSET = 5,
            ADDR_X_ACC_OFFSET = 7,
            ADDR_Y_ACC_OFFSET = 9,
            ADDR_Z_ACC_OFFSET = 11
        };

    public:
        virtual int16_t getXGyroOffset() override;
        virtual int16_t getYGyroOffset() override;
        virtual int16_t getZGyroOffset() override;
        virtual int16_t getXAccOffset() override;
        virtual int16_t getYAccOffset() override;
        virtual int16_t getZAccOffset() override;

        virtual void setXGyroOffset(int16_t offset) override;
        virtual void setYGyroOffset(int16_t offset) override;
        virtual void setZGyroOffset(int16_t offset) override;
        virtual void setXAccOffset(int16_t offset) override;
        virtual void setYAccOffset(int16_t offset) override;
        virtual void setZAccOffset(int16_t offset) override;

        virtual bool getCalibrationFlag() override;
        virtual void setCalibrationFlag() override;
        virtual void resetCalibrationFlag() override;

    private:
        template <typename T>
        T readVal(int address);
};