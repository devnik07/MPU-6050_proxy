#pragma once
class IMUReader {
    public:
        virtual void init() = 0;
        virtual void getRollPitchYaw(float& r, float& p, float& y) = 0;
        virtual ~IMUReader() {};
};