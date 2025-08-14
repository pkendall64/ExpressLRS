#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
#include "gyro.h"
#include "gyro_mpu6050.h"
#include "mixer.h"
#include "logging.h"
#include "MPU6050.h"

extern boolean i2c_enabled;

Gyro gyro;

static bool initialize()
{
    if (!i2c_enabled)
    {
        return false;
    }
    MPU6050 mpu;
    if (mpu.testConnection())
    {
        gyro.dev = new GyroDevMPU6050();
        gyro.dev->initialize();
        DBGLN("Detected MPU6050 Gyro");
        gyro.initialized = true;
        return true;
    }
    return false;
}

static int start()
{
    mixer_initialize();
    if (config.GetCalibrateGyro()) {
        gyro.dev->calibrate();
    }
    return gyro.dev->start(false);
}

static int timeout()
{
    if (gyro.dev->read()) {
        gyro.last_update = micros();
        gyro.send_telemetry();
    }
    gyro.tick();
    return DURATION_IMMEDIATELY;
}

extern bool auto_subtrim_complete;
extern uint8_t subtrim_init;

static int event()
{
    switch (gyro_event)
    {
    case GYRO_EVENT_CALIBRATE:
        gyro_event = GYRO_EVENT_NONE;
        gyro.dev->calibrate();
        config.SetCalibrateGyro(false);
        break;

    case GYRO_EVENT_SUBTRIMS:
        gyro_event = GYRO_EVENT_NONE;
        auto_subtrim_complete = false;
        subtrim_init = 0;
        break;

    default: ;
    }

    return DURATION_IGNORE;
}

device_t Gyro_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONFIG_GYRO_CHANGE,
};
#endif
