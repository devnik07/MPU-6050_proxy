#pragma once

#include "IStorageManager.h"

class FlashManager : public IStorageManager<float> {
    private:
        static constexpr const char* KEY_CALIBRATION_FLAG = "/kv/calibration_flag";
        static constexpr const char* KEY_X_GYRO_OFFSET = "/kv/x_gyro_offset";
        static constexpr const char* KEY_Y_GYRO_OFFSET = "/kv/y_gyro_offset";
        static constexpr const char* KEY_Z_GYRO_OFFSET = "/kv/z_gyro_offset";
        static constexpr const char* KEY_X_ACC_OFFSET = "/kv/x_acc_offset";
        static constexpr const char* KEY_Y_ACC_OFFSET = "/kv/y_acc_offset";
        static constexpr const char* KEY_Z_ACC_OFFSET = "/kv/z_acc_offset";

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
        void put(const char* key, const T& value);

        template <typename T>
        void get (const char* key, T& value);

        template <typename T>
        T readVal(const char* key);
};
