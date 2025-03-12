#pragma once
class IIMUReader {
    public:
        virtual void init() = 0;
        virtual void getRollPitchYaw(float& r, float& p, float& y) = 0;
        virtual ~IIMUReader() {};

        // Calibration
        virtual void calibrate() = 0;
        virtual bool isCalibrated() = 0;
        virtual void resetCalibrationFlag() = 0;
};