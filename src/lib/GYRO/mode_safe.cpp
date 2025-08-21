#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_safe.h"

/**
 * Airplane Safe Mode
 *
 * This allows normal flying but tries to stop the plane going past set angles.
 */

#define PI_180 0.0174532925199

constexpr float max_angle_roll = 30 * PI_180; // Convert degrees to radians
constexpr float max_angle_pitch = 30 * PI_180; // Convert degrees to radians

void SafeController::initialize()
{
    // Set limits to two to be able to fully override a full stick input command
    // on roll and pitch axes
    configure_pids(2.0, 2.0, 1.0);
}

static void _calculate_pid(PID *pid, float angle, float max_angle)
{
    if (abs(angle) < max_angle) {
        pid->reset();
    } else {
        const float setpoint = angle > 0 ? max_angle : - max_angle;
        pid->calculate(setpoint, angle);
    }
}

void SafeController::update()
{
    _calculate_pid(&pid_roll, gyro.f_angle[GYRO_AXIS_ROLL], config.GetGyroSAFERoll() * PI_180);
    _calculate_pid(&pid_pitch, -gyro.f_angle[GYRO_AXIS_PITCH], config.GetGyroSAFEPitch() * PI_180);
    pid_yaw.calculate(0, -gyro.f_gyro[GYRO_AXIS_YAW]);

    setOutput(GYRO_AXIS_ROLL, pid_roll.output);
    setOutput(GYRO_AXIS_PITCH, pid_pitch.output);
    setOutput(GYRO_AXIS_YAW, pid_yaw.output);
}
#endif
