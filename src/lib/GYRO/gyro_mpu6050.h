#pragma once

#include "gyro_device.h"

class GyroDevMPU6050 final : public GyroDevice
{
public:
    uint8_t start(bool calibrate) override;
    bool read() override;
    bool initialize() override;
    void calibrate() override;
};
