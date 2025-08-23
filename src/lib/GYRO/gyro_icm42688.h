#pragma once

#include "ICM42688.h"
#include "Fusion.h"
#include "gyro_device.h"
#include "registers.h"

class ICM42688_INT : public ICM42688
{
public:
    ICM42688_INT(TwoWire &bus, uint8_t address) : ICM42688(bus, address)
    {
    }

    bool isDataReady()
    {
        uint8_t status;
        readRegisters(ICM42688reg::UB0_REG_INT_STATUS, 1, &status);
        return status & 0x08;
    }
};

class GyroDevICM42688 final : public GyroDevice
{
public:
    bool initialize() override;
    uint8_t start(bool calibrate) override;
    bool read() override;
    void calibrate() override;

private:
    ICM42688_INT mpu = ICM42688_INT(Wire, 0x68);
    unsigned long last_update = 0;
    FusionAhrs fusion {};
};
