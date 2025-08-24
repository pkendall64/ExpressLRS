#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_hover.h"

/**
 * Airplane Hover Mode
 *
 * For hover mode we care about two angles
 *
 * 1. The "pitch" angle of the horizon.
 *    This is the amount of error.
 *
 * 2. Rotation around the "roll" axis.
 *    This is the modulation between elevator and rudder to correct the error.
 *
 * With these angles we can command elevator and rudder to correct towards a nose
 * directly up attitude.
 */

void HoverController::initialize() {
    configure_pids(1.0, 1.0, 1.0);
}

void HoverController::update()
{
    // pid_roll.calculate(0, gyro.f_gyro[GYRO_AXIS_ROLL]);
    // pid_pitch.calculate(0, gyro.f_gyro[GYRO_AXIS_PITCH]);
    // pid_yaw.calculate(0, -gyro.f_gyro[GYRO_AXIS_YAW]);
    //
    // float error = gyro.f_angle[GYRO_AXIS_PITCH] - M_PI_2; // Pi/2 = 90degrees
    // error *= (float) config.GetGyroHoverStrength() / 16;
    // pid_pitch.output += error * cos(gyro.f_angle[GYRO_AXIS_ROLL]);
    // pid_yaw.output += error * sin(gyro.f_angle[GYRO_AXIS_ROLL]);
    //
    // // Limit correction as set from gain input channel and
    // // modulate the correction depending on how much axis stick command
    // // FIXME this is using the output (mixed) channel data rather than the stick input!
    // setOutput(GYRO_AXIS_ROLL, pid_roll.output * gyro.gain * (1 - fabs(getChannelData(GYRO_AXIS_ROLL))));
    // setOutput(GYRO_AXIS_PITCH, pid_pitch.output * gyro.gain * (1 - fabs(getChannelData(GYRO_AXIS_PITCH))));
    // setOutput(GYRO_AXIS_YAW, pid_yaw.output * gyro.gain * (1 - fabs(getChannelData(GYRO_AXIS_YAW))));
}

#endif
