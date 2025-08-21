#pragma once

#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
#include "targets.h"
#include "config.h"
#include "gyro_types.h"
#include "pid.h"

#define GYRO_US_MIN 988
#define GYRO_US_MID 1500
#define GYRO_US_MAX 2012

/**
 * Add some servo jitter feedback to the pilot after the gyro has initialized.
 */
#define GYRO_BOOT_JITTER
#ifdef GYRO_BOOT_JITTER
#define GYRO_BOOT_JITTER_US 45
#define GYRO_BOOT_JITTER_MS 175
#define GYRO_BOOT_JITTER_TIMES 4
#endif

class GyroDevice;

extern PID pid_roll;
extern PID pid_pitch;
extern PID pid_yaw;

extern volatile gyro_event_t gyro_event;

class Gyro
{
public:
    void mixer(uint8_t ch, uint16_t *us);
    void send_telemetry();
    void tick();
    void calibrate();
    void reload();

    GyroDevice *dev = nullptr;

    float gain = 1.0;
// protected:

    // orientation/motion vars
    float f_gyro[3];    // roll/pitch/yaw rates radians/s
    float ypr[3];       // [yaw, pitch, roll]   angle in degrees

    uint16_t update_rate;
    unsigned long last_update;
    bool initialized;

private:
    gyro_mode_t gyro_mode = GYRO_MODE_OFF;
    void detect_mode(uint16_t us);
    void switch_mode(gyro_mode_t mode);

    unsigned long pid_delay = 0;
};

extern Gyro gyro;

#endif