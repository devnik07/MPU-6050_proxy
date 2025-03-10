#pragma once

#include "IeepromMPU.h"

class ArduinoEEPROMManager : public IeepromMPU {
    private:
        const int ADDR_CALIBRATION_FLAG = 0;

        enum OffsetAddr {
            ADDR_X_GYRO_OFFSET = 1,
            ADDR_Y_GYRO_OFFSET = 5,
            ADDR_Z_GYRO_OFFSET = 9,
            ADDR_X_ACC_OFFSET = 13,
            ADDR_Y_ACC_OFFSET = 17,
            ADDR_Z_ACC_OFFSET = 21
        };

    public:
        virtual float getXGyroOffset() override;
        virtual float getYGyroOffset() override;
        virtual float getZGyroOffset() override;
        virtual float getXAccOffset() override;
        virtual float getYAccOffset() override;
        virtual float getZAccOffset() override;

        virtual void setXGyroOffset(float offset) override;
        virtual void setYGyroOffset(float offset) override;
        virtual void setZGyroOffset(float offset) override;
        virtual void setXAccOffset(float offset) override;
        virtual void setYAccOffset(float offset) override;
        virtual void setZAccOffset(float offset) override;

        virtual bool getCalibrationFlag() override;
        virtual void setCalibrationFlag() override;
        virtual void resetCalibrationFlag() override;

    private:
        template <typename T>
        T readVal(int address);
};