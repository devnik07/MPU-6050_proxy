#pragma once

class IeepromMPU {
    public:
        virtual int getXGyroOffset() = 0;
        virtual int getYGyroOffset() = 0;
        virtual int getZGyroOffset() = 0;
        virtual int getXAccOffset() = 0;
        virtual int getYAccOffset() = 0;
        virtual int getZAccOffset() = 0;

        virtual void setXGyroOffset(int offset) = 0;
        virtual void setYGyroOffset(int offset) = 0;
        virtual void setZGyroOffset(int offset) = 0;
        virtual void setXAccOffset(int offset) = 0;
        virtual void setYAccOffset(int offset) = 0;
        virtual void setZAccOffset(int offset) = 0;

        virtual bool getCalibrationFlag() = 0;
        virtual void setCalibrationFlag() = 0;
        virtual void resetCalibrationFlag() = 0;

        virtual ~IeepromMPU() {}
};