#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_safe.h"
#include "Fusion.h"
#include <math.h>

void SafeController::initialize() {
    prev_us = 0;
    roll_smooth.reset();
    pitch_smooth.reset();
}

void SafeController::update()
{
    const uint32_t now_us = micros();
    const float dt = (prev_us == 0) ? 0.002f : fmaxf(1e-3f, (now_us - prev_us) * 1e-6f);
    prev_us = now_us;

    // measured angles from quaternion (no Euler sequence)
    float roll_rad, pitch_rad;
    quatToRollPitch(gyro.quaternion, roll_rad, pitch_rad);

    const float roll_max_rad  = (float)config.GetGyroSAFERoll()  * (float)M_PI / 180.0f;
    const float pitch_max_rad = (float)config.GetGyroSAFEPitch() * (float)M_PI / 180.0f;

    // boundary PID (your simple logic)
    auto calc_guard = [](PID& pid, float angle_rad, float max_rad) {
        if (fabsf(angle_rad) < max_rad) { pid.reset(); pid.output = 0.0f; }
        else {
            const float sp = (angle_rad > 0.0f) ? +max_rad : -max_rad;
            pid.calculate(sp, angle_rad);
        }
    };
    calc_guard(pid_roll,  roll_rad,  roll_max_rad);
    calc_guard(pid_pitch, pitch_rad, pitch_max_rad);

    // ignore outward stick at/over the limit (inward always allowed)
    auto outward = [](float angle, float stick)->bool {
        return (angle >= 0.0f && stick > 0.0f) || (angle < 0.0f && stick < 0.0f);
    };
    auto stick_if_allowed = [&](float ang, float lim, float s)->float {
        if (fabsf(ang) >= lim && outward(ang, s)) return 0.0f;
        return s;
    };

    const float stick_roll  = stick_if_allowed(roll_rad,  roll_max_rad,  get_command(GYRO_AXIS_ROLL));
    const float stick_pitch = stick_if_allowed(pitch_rad, pitch_max_rad, -get_command(GYRO_AXIS_PITCH));
    const float yaw_cmd     = get_command(GYRO_AXIS_YAW);

    // scale only the stabilizer by gyro.gain and smooth near limits (via members)
    const float out_roll_raw  = stick_roll  + gyro.gain * pid_roll.output;
    const float out_pitch_raw = stick_pitch + gyro.gain * pid_pitch.output;

    const float out_roll  = roll_smooth.process (roll_rad,  roll_max_rad,  out_roll_raw,  dt);
    const float out_pitch = pitch_smooth.process(pitch_rad, pitch_max_rad, out_pitch_raw, dt);

    setOutputRaw(GYRO_AXIS_ROLL,  out_roll);
    setOutputRaw(GYRO_AXIS_PITCH, out_pitch);

    // yaw damper + RC passthrough, scaled by gyro.gain
    pid_yaw.calculate(0.0f, -gyro.f_gyro[GYRO_AXIS_YAW]);
    setOutputRaw(GYRO_AXIS_YAW,  gyro.gain * pid_yaw.output + yaw_cmd);
}

#endif
