#pragma once

class IeepromMPU {
    public:
        virtual float getXGyroOffset() = 0;
        virtual float getYGyroOffset() = 0;
        virtual float getZGyroOffset() = 0;
        virtual float getXAccOffset() = 0;
        virtual float getYAccOffset() = 0;
        virtual float getZAccOffset() = 0;

        virtual void setXGyroOffset(float offset) = 0;
        virtual void setYGyroOffset(float offset) = 0;
        virtual void setZGyroOffset(float offset) = 0;
        virtual void setXAccOffset(float offset) = 0;
        virtual void setYAccOffset(float offset) = 0;
        virtual void setZAccOffset(float offset) = 0;

        virtual bool getCalibrationFlag() = 0;
        virtual void setCalibrationFlag() = 0;
        virtual void resetCalibrationFlag() = 0;

        virtual ~IeepromMPU() {}
};