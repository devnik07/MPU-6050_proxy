#include "ArduinoEEPROMManager.h"

#include <EEPROM.h>

template <typename T>
T ArduinoEEPROMManager::readVal(int address) {
    T val;
    EEPROM.get(address, val);
    return val;
}

float ArduinoEEPROMManager::getXGyroOffset() {
    return readVal<float>(ADDR_X_GYRO_OFFSET);
}

float ArduinoEEPROMManager::getYGyroOffset() {
    return readVal<float>(ADDR_Y_GYRO_OFFSET);
}

float ArduinoEEPROMManager::getZGyroOffset() {
    return readVal<float>(ADDR_Z_GYRO_OFFSET);
}

float ArduinoEEPROMManager::getXAccOffset() {
    return readVal<float>(ADDR_X_ACC_OFFSET);
}

float ArduinoEEPROMManager::getYAccOffset() {
    return readVal<float>(ADDR_Y_ACC_OFFSET);
}

float ArduinoEEPROMManager::getZAccOffset() {
    return readVal<float>(ADDR_Z_ACC_OFFSET);
}

void ArduinoEEPROMManager::setXGyroOffset(float offset) {
    EEPROM.put(ADDR_X_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setYGyroOffset(float offset) {
    EEPROM.put(ADDR_Y_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setZGyroOffset(float offset) {
    EEPROM.put(ADDR_Z_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setXAccOffset(float offset) {
    EEPROM.put(ADDR_X_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setYAccOffset(float offset) {
    EEPROM.put(ADDR_Y_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setZAccOffset(float offset) {
    EEPROM.put(ADDR_Z_ACC_OFFSET, offset);
}

bool ArduinoEEPROMManager::getCalibrationFlag() {
    return readVal<bool>(ADDR_CALIBRATION_FLAG);
}

void ArduinoEEPROMManager::setCalibrationFlag() {
    EEPROM.put(ADDR_CALIBRATION_FLAG, true);
}

void ArduinoEEPROMManager::resetCalibrationFlag() {
    EEPROM.put(ADDR_CALIBRATION_FLAG, false);
}
