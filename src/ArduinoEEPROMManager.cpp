#include "ArduinoEEPROMManager.h"

#include <EEPROM.h>

template <typename T>
T ArduinoEEPROMManager::readVal(int address) {
    T val;
    EEPROM.get(address, val);
    return val;
}

int16_t ArduinoEEPROMManager::getXGyroOffset() {
    return readVal<int16_t>(ADDR_X_GYRO_OFFSET);
}

int16_t ArduinoEEPROMManager::getYGyroOffset() {
    return readVal<int16_t>(ADDR_Y_GYRO_OFFSET);
}

int16_t ArduinoEEPROMManager::getZGyroOffset() {
    return readVal<int16_t>(ADDR_Z_GYRO_OFFSET);
}

int16_t ArduinoEEPROMManager::getXAccOffset() {
    return readVal<int16_t>(ADDR_X_ACC_OFFSET);
}

int16_t ArduinoEEPROMManager::getYAccOffset() {
    return readVal<int16_t>(ADDR_Y_ACC_OFFSET);
}

int16_t ArduinoEEPROMManager::getZAccOffset() {
    return readVal<int16_t>(ADDR_Z_ACC_OFFSET);
}

int16_t ArduinoEEPROMManager::getXMagOffset() {
    return 0; // Not implemented
}

int16_t ArduinoEEPROMManager::getYMagOffset() {
    return 0; // Not implemented
}

int16_t ArduinoEEPROMManager::getZMagOffset() {
    return 0; // Not implemented
}

void ArduinoEEPROMManager::setXGyroOffset(int16_t offset) {
    EEPROM.put(ADDR_X_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setYGyroOffset(int16_t offset) {
    EEPROM.put(ADDR_Y_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setZGyroOffset(int16_t offset) {
    EEPROM.put(ADDR_Z_GYRO_OFFSET, offset);
}

void ArduinoEEPROMManager::setXAccOffset(int16_t offset) {
    EEPROM.put(ADDR_X_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setYAccOffset(int16_t offset) {
    EEPROM.put(ADDR_Y_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setZAccOffset(int16_t offset) {
    EEPROM.put(ADDR_Z_ACC_OFFSET, offset);
}

void ArduinoEEPROMManager::setXMagOffset(int16_t offset) {
    return; // Not implemented
}

void ArduinoEEPROMManager::setYMagOffset(int16_t offset) {
    return; // Not implemented
}

void ArduinoEEPROMManager::setZMagOffset(int16_t offset) {
    return; // Not implemented
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
