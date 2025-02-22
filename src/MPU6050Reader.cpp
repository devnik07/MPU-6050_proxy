#include "MPU6050Reader.h"

MPU6050Reader::MPU6050Reader(MPU6050& mpu6050) : mpu(mpu6050) {}

void MPU6050Reader::init() {
    uint8_t devStatus;

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();

    /* Initialize and configure the DMP */
    devStatus = mpu.dmpInitialize();
    setStartingOffsets();
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
    }
}

void MPU6050Reader::getRollPitchYaw(float& r, float& p, float& y) {
    Quaternion q;

    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        float q0 = q.w;
        float q1 = q.x;
        float q2 = q.y;
        float q3 = q.z;

        float rr = atan2(-2*q1*q3 + 2*q0*q2, q3*q3 - q2*q2 - q1*q1 + q0*q0);
        float pr = asin(2*q2*q3 + 2*q0*q1);
        float yr = -atan2(-2*q1*q2 + 2*q0*q3, q2*q2 - q3*q3 - q1*q1 + q0*q0);

        r = rr * 180/M_PI;
        p = pr * 180/M_PI;
        y = yr * 180/M_PI;
    }
}

void MPU6050Reader::setStartingOffsets() {
    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

}