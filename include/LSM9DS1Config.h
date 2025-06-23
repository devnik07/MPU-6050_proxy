#pragma once

#include "IConfig.h"
#include "FlashManager.h"
#include "LSM9DS1Reader.h"

class LSM9DS1Config : public IConfig {
    private:
        FlashManager flashManager;
        LSM9DS1Reader reader;

    public:
        LSM9DS1Config(Madgwick& filter);
        IIMUReader& getReader() override;
};