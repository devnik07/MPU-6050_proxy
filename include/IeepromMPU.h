#pragma once

#include <stdint.h>

class IeepromMPU {
    public:
        virtual int16_t getXGyroOffset() = 0;
        virtual int16_t getYGyroOffset() = 0;
        virtual int16_t getZGyroOffset() = 0;
        virtual int16_t getXAccOffset() = 0;
        virtual int16_t getYAccOffset() = 0;
        virtual int16_t getZAccOffset() = 0;

        virtual void setXGyroOffset(int16_t offset) = 0;
        virtual void setYGyroOffset(int16_t offset) = 0;
        virtual void setZGyroOffset(int16_t offset) = 0;
        virtual void setXAccOffset(int16_t offset) = 0;
        virtual void setYAccOffset(int16_t offset) = 0;
        virtual void setZAccOffset(int16_t offset) = 0;

        virtual bool getCalibrationFlag() = 0;
        virtual void setCalibrationFlag() = 0;
        virtual void resetCalibrationFlag() = 0;

        virtual ~IeepromMPU() {}
};