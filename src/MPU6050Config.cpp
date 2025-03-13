#include "MPU6050Config.h"

MPU6050Config::MPU6050Config(MPU6050& mpu) : reader(eepromManager, mpu) {}

IIMUReader& MPU6050Config::getReader() {
    return reader;
}