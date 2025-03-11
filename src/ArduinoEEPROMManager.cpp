#include "ArduinoEEPROMManager.h"

#include <EEPROM.h>

template <typename T>
T ArduinoEEPROMManager::readVal(int address) {
    T val;
    EEPROM.get(address, val);
    return val;
}

int ArduinoEEPROMManager::getXGyroOffset() {
    return readVal<int>(ADDR_X_GYRO_OFFSET);
}

int ArduinoEEPROMManager::getYGyroOffset() {
    return readVal<int>(ADDR_Y_GYRO_OFFSET);
}

int ArduinoEEPROMManager::getZGyroOffset() {
    return readVal<int>(ADDR_Z_GYRO_OFFSET);
}

int ArduinoEEPROMManager::getXAccOffset() {
    return readVal<int>(ADDR_X_ACC_OFFSET);
}

int ArduinoEEPROMManager::getYAccOffset() {
    return readVal<int>(ADDR_Y_ACC_OFFSET);
}

int ArduinoEEPROMManager::getZAccOffset() {
    return readVal<int>(ADDR_Z_ACC_OFFSET);
}

void ArduinoEEPROMManager::setXGyroOffset(int offset) {
    EEPROM.put(ADDR_X_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setYGyroOffset(int offset) {
    EEPROM.put(ADDR_Y_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setZGyroOffset(int offset) {
    EEPROM.put(ADDR_Z_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setXAccOffset(int offset) {
    EEPROM.put(ADDR_X_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setYAccOffset(int offset) {
    EEPROM.put(ADDR_Y_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setZAccOffset(int offset) {
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
