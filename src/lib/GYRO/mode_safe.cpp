#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_safe.h"
#include "Fusion.h"
#include <math.h>

void SafeController::initialize()
{
    // Same PID setup pattern as Rate mode; limits are set via config gains
    configure_pids(2.0f, 2.0f, 1.0f);
}

void SafeController::update()
{
    const uint32_t now_us = micros();
    const float dt = (prev_us == 0) ? 0.002f : fmaxf(1e-3f, (now_us - prev_us) * 1e-6f);
    prev_us = now_us;

    // 1) Attitude from quaternion (no Euler)
    float roll_rad, pitch_rad;
    quatToRollPitch(gyro.quaternion, roll_rad, pitch_rad);

    // 2) Limits (deg → rad)
    const float roll_max_rad  = (float)config.GetGyroSAFERoll()  * (float)M_PI / 180.0f;
    const float pitch_max_rad = (float)config.GetGyroSAFEPitch() * (float)M_PI / 180.0f;

    // 3) Boundary PID (your original simple logic)
    auto calc_guard = [](PID& pid, float angle_rad, float max_rad) {
        if (fabsf(angle_rad) < max_rad) { pid.reset(); pid.output = 0.0f; }
        else {
            const float sp = (angle_rad > 0.0f) ? +max_rad : -max_rad;
            pid.calculate(sp, angle_rad);
        }
    };
    calc_guard(pid_roll,  roll_rad,  roll_max_rad);
    calc_guard(pid_pitch, pitch_rad, pitch_max_rad);

    // 4) Ignore outward stick once at/over the limit
    const float stick_roll  = stickIfAllowed(roll_rad,  roll_max_rad,  get_command(GYRO_AXIS_ROLL));
    const float stick_pitch = stickIfAllowed(pitch_rad, pitch_max_rad, get_command(GYRO_AXIS_PITCH));

    // 5) Compose and smooth near the edge (prevents servo slamming in gusts)
    const float out_roll_raw  = stick_roll  + pid_roll.output;
    const float out_pitch_raw = stick_pitch + pid_pitch.output;

    const float out_roll  = roll_smooth.process (roll_rad,  roll_max_rad,  out_roll_raw,  dt);
    const float out_pitch = pitch_smooth.process(pitch_rad, pitch_max_rad, out_pitch_raw, dt);

    // 6) Outputs
    setOutputRaw(GYRO_AXIS_ROLL,  out_roll);
    setOutputRaw(GYRO_AXIS_PITCH, out_pitch);

    // Yaw: unchanged (rate damper + RC added inside)
    pid_yaw.calculate(0.0f, -gyro.f_gyro[GYRO_AXIS_YAW]);
    setOutputRaw(GYRO_AXIS_YAW, pid_yaw.output + get_command(GYRO_AXIS_YAW));
}

#endif
