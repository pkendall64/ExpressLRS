#if defined(PLATFORM_ESP32) && defined(TARGET_RX) && defined(DEBUG_GYRO_STATS)
#include "gyro_device.h"

#include "gyro.h"
#include "gyro_types.h"
#include "logging.h"

#include <esp32-hal.h>

/**
 * For debugging print useful gyro state
 */
void GyroDevice::print_gyro_stats()
{
    if (millis() - last_gyro_stats_time < 500)
        return;

    // Calculate gyro update rate in HZ
    int update_rate = 1.0 /
                      ((micros() - gyro.last_update) / 1000000.0);

    char rate_str[5]; sprintf(rate_str, "%4d", update_rate);

    char roll_str[8]; sprintf(roll_str, "%6.2f", gyro.f_angle.angle.roll);
    char pitch_str[8]; sprintf(pitch_str, "%6.2f", gyro.f_angle.angle.pitch);
    char yaw_str[8]; sprintf(yaw_str, "%6.2f", gyro.f_angle.angle.yaw);

    char gyro_x[8]; sprintf(gyro_x, "%6.2f", gyro.f_gyro[GYRO_AXIS_ROLL]);
    char gyro_y[8]; sprintf(gyro_y, "%6.2f", gyro.f_gyro[GYRO_AXIS_PITCH]);
    char gyro_z[8]; sprintf(gyro_z, "%6.2f", gyro.f_gyro[GYRO_AXIS_YAW]);

    char accel_x[8]; sprintf(accel_x, "%6.2f", gyro.f_accel[GYRO_AXIS_ROLL]);
    char accel_y[8]; sprintf(accel_y, "%6.2f", gyro.f_accel[GYRO_AXIS_PITCH]);
    char accel_z[8]; sprintf(accel_z, "%6.2f", gyro.f_accel[GYRO_AXIS_YAW]);

    DBGLN(
        "%s HZ "
        "Gain %f "
        "Pitch: %s Roll: %s Yaw: %s "
        "Gyro x: %s Gyro y: %s Gyro z: %s "
        "Accel x: %s Accel y: %s Accel z: %s "
        ,rate_str
        ,gyro.gain
        ,pitch_str, roll_str, yaw_str
        ,gyro_x, gyro_y, gyro_z
        ,accel_x, accel_y, accel_z
        );

    last_gyro_stats_time = millis();
}
#endif
