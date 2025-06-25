#include "LSM9DS1Reader.h"

LSM9DS1Reader::LSM9DS1Reader(IStorageManager<float>& iflashManager, Madgwick& filter)
                            : flashManager(iflashManager), filter(filter) {
    gyroXOffset = 0;
    gyroYOffset = 0;
    gyroZOffset = 0;
    accXOffset = 0;
    accYOffset = 0;
    accZOffset = 0;
    magXOffset = 0;
    magYOffset = 0;
    magZOffset = 0;
}

void LSM9DS1Reader::init() {
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU.");
        while (true);   // Stop execution
    } else {
        if (isCalibrated()) {
            loadCalibrationOffsets();
        } else {
            calibrate();
        }
    }
}

void LSM9DS1Reader::getRollPitchYaw(float& r, float& p, float& y) {
    float gForceX, gForceY, gForceZ; 
    float rateRoll, ratePitch, rateYaw;
    float mX, mY, mZ;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        IMU.readAcceleration(gForceX, gForceY, gForceZ);
        IMU.readGyroscope(rateRoll, ratePitch, rateYaw);
        IMU.readMagneticField(mX, mY, mZ);

        if (isCalibrated()) {
            gForceX -= accXOffset;
            gForceY -= accYOffset;
            gForceZ -= accZOffset;
            rateRoll -= gyroXOffset;
            ratePitch -= gyroYOffset;
            rateYaw -= gyroZOffset;
            mX -= magXOffset;
            mY -= magYOffset;
            mZ -= magZOffset;
        }

        filter.update(rateRoll, ratePitch, rateYaw,
                      gForceX, gForceY, gForceZ, 
                      mX, mY, mZ);
        r = filter.getRoll();
        p = filter.getPitch();
        y = filter.getYaw();
    }
}

void LSM9DS1Reader::getRotationQuaternion(float& w, float& x, float& y, float& z) {
    float gForceX, gForceY, gForceZ; 
    float rateRoll, ratePitch, rateYaw;
    float mX, mY, mZ;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        IMU.readAcceleration(gForceX, gForceY, gForceZ);
        IMU.readGyroscope(rateRoll, ratePitch, rateYaw);
        IMU.readMagneticField(mX, mY, mZ);

        if (isCalibrated()) {
            gForceX -= accXOffset;
            gForceY -= accYOffset;
            gForceZ -= accZOffset;
            rateRoll -= gyroXOffset;
            ratePitch -= gyroYOffset;
            rateYaw -= gyroZOffset;
            mX -= magXOffset;
            mY -= magYOffset;
            mZ -= magZOffset;
        }

        filter.update(rateRoll, ratePitch, rateYaw,
                      gForceX, gForceY, gForceZ, 
                      mX, mY, mZ);
        float roll = filter.getRollRadians();
        float pitch = filter.getPitchRadians();
        float yaw = filter.getYawRadians();

        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);

        w = cr*cp*cy + sr*sp*sy;
        x = sr*cp*cy - cr*sp*sy;
        y = cr*sp*cy + sr*cp*sy;
        z = cr*cp*sy - sr*sp*cy;
    }
}

void LSM9DS1Reader::calibrate() {
    Serial.println("Starting calibration.");

    calibrateGyro();
    calibrateAccel();
    calibrateMag();

    // Save offsets
    flashManager.setXGyroOffset(gyroXOffset);
    flashManager.setYGyroOffset(gyroYOffset);
    flashManager.setZGyroOffset(gyroZOffset);

    flashManager.setXAccOffset(accXOffset);
    flashManager.setYAccOffset(accYOffset);
    flashManager.setZAccOffset(accZOffset);

    flashManager.setXMagOffset(magXOffset);
    flashManager.setYMagOffset(magYOffset);
    flashManager.setZMagOffset(magZOffset);

    flashManager.setCalibrationFlag();

    Serial.println("Calibration completed.");
}

bool LSM9DS1Reader::isCalibrated() {
    return flashManager.getCalibrationFlag();
}

void LSM9DS1Reader::resetCalibrationFlag() {
    flashManager.resetCalibrationFlag();
    Serial.println("Calibration flag reset.");
}

void LSM9DS1Reader::loadCalibrationOffsets() {
    Serial.println("Loading Calibration Offsets.");

    gyroXOffset = flashManager.getXGyroOffset();
    gyroYOffset = flashManager.getYGyroOffset();
    gyroZOffset = flashManager.getZGyroOffset();
    
    accXOffset = flashManager.getXAccOffset();
    accYOffset = flashManager.getYAccOffset();
    accZOffset = flashManager.getZAccOffset();

    magXOffset = flashManager.getXMagOffset();
    magYOffset = flashManager.getYMagOffset();
    magZOffset = flashManager.getZMagOffset();

    Serial.print("Acc X: ");
    Serial.println(accXOffset);
    Serial.print("Acc Y: ");
    Serial.println(accYOffset);
    Serial.print("Acc Z: ");
    Serial.println(accZOffset);
    Serial.print("Gyro X: ");
    Serial.println(gyroXOffset);
    Serial.print("Gyro Y: ");
    Serial.println(gyroYOffset);
    Serial.print("Gyro Z: ");
    Serial.println(gyroZOffset);
    Serial.print("Mag X: ");
    Serial.println(magXOffset);
    Serial.print("Mag Y: ");
    Serial.println(magYOffset);
    Serial.print("Mag Z: ");
    Serial.println(magZOffset);

    Serial.println("Calibration Offsets loaded.");
}

/*
  Computes offset values for the Gyroscope.
*/
void LSM9DS1Reader::calibrateGyro() {
    Serial.println("Calibrating Accelerometer ...");

    int numSamples = 2000;
    float rateRoll, ratePitch, rateYaw;

    for (int i = 0; i < numSamples; i++) {
        IMU.readGyroscope(rateRoll, ratePitch, rateYaw);
        gyroXOffset += rateRoll;
        gyroYOffset += ratePitch;
        gyroZOffset += rateYaw;
        delay(1);
    }

    gyroXOffset /= numSamples;
    gyroYOffset /= numSamples;
    gyroZOffset /= numSamples;

    Serial.println("Gyroscope Calibration completed.");
}

/*
  Compute offset values for the Accelerometer.
*/
void LSM9DS1Reader::calibrateAccel() {
    Serial.println("Calibrating Accelerometer ...");

    int numSamples = 2000;
    float gForceX, gForceY, gForceZ;

    for (int i = 0; i < numSamples; i++) {
        IMU.readAcceleration(gForceX, gForceY, gForceZ);
        accXOffset += gForceX;
        accYOffset += gForceY;
        accZOffset += gForceZ;
        delay(1);
    }

    accXOffset /= numSamples;
    accYOffset /= numSamples;
    accZOffset = (accZOffset / numSamples) - 1;

    Serial.println("Accelerometer Calibration completed.");
}

void LSM9DS1Reader::calibrateMag() {
    Serial.println("Calibrating Magnetometer ...");

    int numSamples = 2000;
    float mX, mY, mZ;
    float mMinX, mMinY, mMinZ;
    float mMaxX, mMaxY, mMaxZ;

    mMinX = mMinY = mMinZ = 1e6;
    mMaxX = mMaxY = mMaxZ = -1e6;

    for (int i = 0; i < numSamples; i++) {
        IMU.readMagneticField(mX, mY, mZ);

        if (mX < mMinX) mMinX = mX;
        if (mY < mMinY) mMinY = mY;
        if (mZ < mMinZ) mMinZ = mZ;

        if (mX > mMaxX) mMaxX = mX;
        if (mY > mMaxY) mMaxY = mY;
        if (mZ > mMaxZ) mMaxZ = mZ;

        delay(1);
    }

    magXOffset = (mMinX + mMaxX) / 2;
    magYOffset = (mMinY + mMaxY) / 2;
    magZOffset = (mMinZ + mMaxZ) / 2;

    Serial.println("Magnetometer calibration completed.");
}