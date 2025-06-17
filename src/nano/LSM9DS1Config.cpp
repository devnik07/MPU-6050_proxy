#include "LSM9DS1Config.h"

LSM9DS1Config::LSM9DS1Config(Madgwick& filter) : reader(filter) {}

IIMUReader& LSM9DS1Config::getReader() {
    return reader;
}