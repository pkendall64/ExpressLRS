#ifndef SRC_GYRO_DEVICE_H
#define SRC_GYRO_DEVICE_H

#include <cstdint>

class GyroDevice
{
public:
    virtual ~GyroDevice() = default;
    virtual bool initialize() { return true; }
    virtual uint8_t start(bool calibrate) = 0;
    virtual bool read() = 0;
    virtual void calibrate() = 0;
#ifdef DEBUG_GYRO_STATS
    void print_gyro_stats();
    unsigned long last_gyro_stats_time = 0;
#endif
};

#endif // SRC_GYRO_DEVICE_H
