#include "FlashManager.h"

#include "KVStore.h"
#include "kvstore_global_api.h"

/**
 * Read values of various data types.
 */
template <typename T>
void FlashManager::get(const char* key, T& value) {
    size_t actual_size = 0;
    kv_get(key, &value, sizeof(T), &actual_size);
}

/**
 * Write values of various data types.
 */
template <typename T>
void FlashManager::put(const char* key, const T& value) {
    kv_set(key, &value, sizeof(T), 0);
}

template <typename T>
T FlashManager::readVal(const char* key) {
    T val;
    get(key, val);
    return val;
}

float FlashManager::getXGyroOffset() {
    return readVal<float>(KEY_X_GYRO_OFFSET);
}

float FlashManager::getYGyroOffset() {
    return readVal<float>(KEY_Y_GYRO_OFFSET);
}

float FlashManager::getZGyroOffset() {
    return readVal<float>(KEY_Z_GYRO_OFFSET);
}

float FlashManager::getXAccOffset() {
    return readVal<float>(KEY_X_ACC_OFFSET);
}

float FlashManager::getYAccOffset() {
    return readVal<float>(KEY_Y_ACC_OFFSET);
}

float FlashManager::getZAccOffset() {
    return readVal<float>(KEY_Z_ACC_OFFSET);
}

void FlashManager::setXGyroOffset(float offset) {
    put<float>(KEY_X_GYRO_OFFSET, offset);
}

void FlashManager::setYGyroOffset(float offset) {
    put<float>(KEY_Y_GYRO_OFFSET, offset);
}

void FlashManager::setZGyroOffset(float offset) {
    put<float>(KEY_Z_GYRO_OFFSET, offset);
}

void FlashManager::setXAccOffset(float offset) {
    put<float>(KEY_X_ACC_OFFSET, offset);
}

void FlashManager::setYAccOffset(float offset) {
    put<float>(KEY_Y_ACC_OFFSET, offset);
}

void FlashManager::setZAccOffset(float offset) {
    put<float>(KEY_Z_ACC_OFFSET, offset);
}

bool FlashManager::getCalibrationFlag() {
    return readVal<bool>(KEY_CALIBRATION_FLAG);
}

void FlashManager::setCalibrationFlag() {
    put<bool>(KEY_CALIBRATION_FLAG, true);
}

void FlashManager::resetCalibrationFlag() {
    put<bool>(KEY_CALIBRATION_FLAG, false);
}