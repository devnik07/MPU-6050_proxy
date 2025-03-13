#include "CustomMPU6050Config.h"

CustomMPU6050Config::CustomMPU6050Config(const ComputationOption compOpt) : reader(eepromManager, compOpt) {}

IIMUReader& CustomMPU6050Config::getReader() {
    return reader;
}