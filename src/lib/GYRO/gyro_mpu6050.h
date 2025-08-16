#pragma once

#include "helper_3dmath.h"

// See betaflight src/main/drivers/accgyro/accgyro_mpu6050.c

class GyroDevice
{
public:
    virtual ~GyroDevice() = default;
    virtual void initialize() {}
    virtual uint8_t start(bool calibrate);
    virtual bool read();
    virtual void calibrate();
};

class GyroDevMPU6050 final : public GyroDevice
{
public:
    uint8_t start(bool calibrate) override;
    bool read() override;
    void calibrate() override;

private:
    void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
    #ifdef DEBUG_GYRO_STATS
    void print_gyro_stats();
    unsigned long last_gyro_stats_time = 0;
    #endif
};
