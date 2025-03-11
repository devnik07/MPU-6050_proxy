#pragma once

#include "IeepromMPU.h"

class ArduinoEEPROMManager : public IeepromMPU {
    private:
        const int ADDR_CALIBRATION_FLAG = 0;

        enum OffsetAddr {
            ADDR_X_GYRO_OFFSET = 1,
            ADDR_Y_GYRO_OFFSET = 3,
            ADDR_Z_GYRO_OFFSET = 5,
            ADDR_X_ACC_OFFSET = 7,
            ADDR_Y_ACC_OFFSET = 9,
            ADDR_Z_ACC_OFFSET = 11
        };

    public:
        virtual int getXGyroOffset() override;
        virtual int getYGyroOffset() override;
        virtual int getZGyroOffset() override;
        virtual int getXAccOffset() override;
        virtual int getYAccOffset() override;
        virtual int getZAccOffset() override;

        virtual void setXGyroOffset(int offset) override;
        virtual void setYGyroOffset(int offset) override;
        virtual void setZGyroOffset(int offset) override;
        virtual void setXAccOffset(int offset) override;
        virtual void setYAccOffset(int offset) override;
        virtual void setZAccOffset(int offset) override;

        virtual bool getCalibrationFlag() override;
        virtual void setCalibrationFlag() override;
        virtual void resetCalibrationFlag() override;

    private:
        template <typename T>
        T readVal(int address);
};