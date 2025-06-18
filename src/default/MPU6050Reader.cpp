#include "MPU6050Reader.h"

MPU6050Reader::MPU6050Reader(IStorageManager<int16_t>& ieepromManager, MPU6050& mpu6050)
                : eepromManager(ieepromManager), mpu(mpu6050) {}

void MPU6050Reader::init() {
    uint8_t devStatus = setupMPU();

    if (devStatus == 0) {
        if (isCalibrated()) {
            loadCalibrationOffsets();
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);
        } else {
            Serial.println("MPU not calibrated. Stopping execution.");
            while (true);   // Stop execution 
        }
    } else {
        Serial.print(F("DMP Initialization failed (code ")); // Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        while(true);    // Stop execution
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

        float rr = atan2(2*(q0*q1 + q2*q3), 1 - 2*(pow(q1, 2) + pow(q2, 2)));
        float pr = -asin(2*(q0*q2 - q3*q1));
        float yr = -atan2(2*(q0*q3 + q1*q2), 1 - 2*(pow(q2, 2) + pow(q3, 2)));

        r = rr * 180/M_PI;
        p = pr * 180/M_PI;
        y = yr * 180/M_PI;
    }
}

void MPU6050Reader::getRotationQuaternion(float& w, float& x, float& y, float& z) {
    Quaternion q;
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        w = q.w;
        x = q.x;
        y = q.y;
        z = q.z;
    }
}

void MPU6050Reader::calibrate() {
    Serial.println("Starting Calibration.");

    uint8_t devStatus = setupMPU();

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        // Save offsets
        eepromManager.setXGyroOffset(mpu.getXGyroOffset());
        eepromManager.setYGyroOffset(mpu.getYGyroOffset());
        eepromManager.setZGyroOffset(mpu.getZGyroOffset());
        
        eepromManager.setXAccOffset(mpu.getXAccelOffset());
        eepromManager.setYAccOffset(mpu.getYAccelOffset());
        eepromManager.setZAccOffset(mpu.getZAccelOffset());

        eepromManager.setCalibrationFlag();

        Serial.println("\nCalibration completed.");
    } else {
        Serial.print(F("DMP Initialization failed (code ")); // Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        while(true);    // Stop execution
    }
}

bool MPU6050Reader::isCalibrated() {
    return eepromManager.getCalibrationFlag();
}

void MPU6050Reader::resetCalibrationFlag() {
    eepromManager.resetCalibrationFlag();
    Serial.println("Calibration flag reset.");
}

void MPU6050Reader::loadCalibrationOffsets() {
    Serial.println("Loading Calibration Offsets.");

    mpu.setXGyroOffset(eepromManager.getXGyroOffset());
    mpu.setYGyroOffset(eepromManager.getYGyroOffset());
    mpu.setZGyroOffset(eepromManager.getZGyroOffset());

    mpu.setXAccelOffset(eepromManager.getXAccOffset());
    mpu.setYAccelOffset(eepromManager.getYAccOffset());
    mpu.setZAccelOffset(eepromManager.getZAccOffset());

    Serial.print("Acc X: ");
    Serial.println(mpu.getXAccelOffset());
    Serial.print("Acc Y: ");
    Serial.println(mpu.getYAccelOffset());
    Serial.print("Acc Z: ");
    Serial.println(mpu.getZAccelOffset());
    Serial.print("Gyro X: ");
    Serial.println(mpu.getXGyroOffset());
    Serial.print("Gyro Y: ");
    Serial.println(mpu.getYGyroOffset());
    Serial.print("Gyro Z: ");
    Serial.println(mpu.getZGyroOffset());

    Serial.println("Calibration Offsets loaded.");
}

uint8_t MPU6050Reader::setupMPU() {
    uint8_t devStatus;

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    /* Initialize and configure the DMP */
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    setStartingOffsets();
    return devStatus;
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