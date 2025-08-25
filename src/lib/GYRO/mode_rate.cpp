#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_rate.h"

/**
 * Airplane Normal Mode / Rate Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases, the correction decreases, allowing unlimited
 * angular rates.
 */

void RateController::initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void RateController::update()
{
    // The desired angular rate is zero
    pid_roll.calculate(0, gyro.f_gyro[GYRO_AXIS_ROLL]);
    pid_pitch.calculate(0, gyro.f_gyro[GYRO_AXIS_PITCH]);
    pid_yaw.calculate(0, -gyro.f_gyro[GYRO_AXIS_YAW]);

    // Limit correction as set from gain input channel and
    // modulate the correction depending on how much axis stick command
    setOutputRaw(GYRO_AXIS_ROLL, pid_roll.output * gyro.gain * (1 - fabs(getChannelData(GYRO_AXIS_ROLL))) + get_command(GYRO_AXIS_ROLL));
    setOutputRaw(GYRO_AXIS_PITCH, pid_pitch.output * gyro.gain * (1 - fabs(getChannelData(GYRO_AXIS_PITCH))) + get_command(GYRO_AXIS_PITCH));
    setOutputRaw(GYRO_AXIS_YAW, pid_yaw.output * gyro.gain * (1 - fabs(getChannelData(GYRO_AXIS_YAW))) + get_command(GYRO_AXIS_YAW));
}
#endif
