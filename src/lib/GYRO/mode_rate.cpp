#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "gyro.h"

/**
 * Airplane Normal Mode / Rate Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases, the correction decreases, allowing unlimited
 * angular rates.
 */

void rate_controller_initialize()
{
    configure_pids(1.0, 1.0, 1.0);

    // For rate mode we have a basic derivative from the gyro which is the
    // angular velocity. Therefor we turn of any derivative term.
    // pid_pitch._Kd = 0;
    // pid_roll._Kd = 0;
    // pid_yaw._Kd = 0;
}

void rate_controller_calculate_pid()
{
    // The desired angular rate is zero
    pid_roll.calculate(0, gyro.f_gyro[0]);
    pid_pitch.calculate(0, gyro.f_gyro[1]);
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
}

#endif
