#pragma once

#include <stdint.h>

template<typename T>
class IStorageManager {
    public:
        virtual T getXGyroOffset() = 0;
        virtual T getYGyroOffset() = 0;
        virtual T getZGyroOffset() = 0;
        virtual T getXAccOffset() = 0;
        virtual T getYAccOffset() = 0;
        virtual T getZAccOffset() = 0;
        virtual T getXMagOffset() = 0;
        virtual T getYMagOffset() = 0;
        virtual T getZMagOffset() = 0;

        virtual void setXGyroOffset(T offset) = 0;
        virtual void setYGyroOffset(T offset) = 0;
        virtual void setZGyroOffset(T offset) = 0;
        virtual void setXAccOffset(T offset) = 0;
        virtual void setYAccOffset(T offset) = 0;
        virtual void setZAccOffset(T offset) = 0;
        virtual void setXMagOffset(T offset) = 0;
        virtual void setYMagOffset(T offset) = 0;
        virtual void setZMagOffset(T offset) = 0;

        virtual bool getCalibrationFlag() = 0;
        virtual void setCalibrationFlag() = 0;
        virtual void resetCalibrationFlag() = 0;

        virtual ~IStorageManager() {}
};