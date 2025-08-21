#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_level.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 */

#define degToRad(angleInDegrees) (float)((angleInDegrees) * M_PI / 180.0)


void LevelController::initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void LevelController::update()
{
    const float roll = get_command(GYRO_AXIS_ROLL);
    pid_roll.calculate(
        roll * degToRad(config.GetGyroLevelRoll()),
        gyro.ypr[2]
    );
    setOutput(GYRO_AXIS_ROLL, pid_roll.output - roll);

    const float pitch = get_command(GYRO_AXIS_PITCH);
    pid_pitch.calculate(
        pitch * degToRad(config.GetGyroLevelPitch()),
        -gyro.ypr[1]
    );
    setOutput(GYRO_AXIS_PITCH, pid_pitch.output - pitch);

    pid_yaw.calculate(0, -gyro.f_gyro[2]);
    setOutput(GYRO_AXIS_YAW, pid_yaw.output);
}

#endif
