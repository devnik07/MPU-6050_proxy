#pragma once

#include "IIMUReader.h"

class IConfig {
    public:
        virtual IIMUReader& getReader() = 0;
        virtual ~IConfig() {}
};