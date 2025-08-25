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
    // === timebase for smoothing / slew ===
    static uint32_t prev_us = 0;
    static float roll_cmd_slew = 0.0f, pitch_cmd_slew = 0.0f; // persisted final commands
    const uint32_t now_us = micros();
    const float dt = (prev_us == 0) ? 0.002f : fmaxf(1e-3f, (now_us - prev_us) * 1e-6f);
    prev_us = now_us;

    // === 1) Attitude from quaternion (no Euler) ===
    const FusionQuaternion q_be = gyro.quaternion;                 // body->earth
    const FusionQuaternion q_eb = FusionQuaternionConjugate(q_be); // earth->body
    const FusionMatrix R = FusionQuaternionToMatrix(q_eb);

    // body-frame gravity = R * [0,0,1]^T  (third column of R)
    const float zx = R.array[0][2];
    const float zy = R.array[1][2];
    const float zz = R.array[2][2];

    // roll/pitch from body gravity (robust in normal envelopes)
    const float roll_rad  = atan2f(zy, zz);
    const float pitch_rad = -atan2f(zx, zz);  // minus matches your original pitch sign

    // === 2) Limits & hysteresis ===
    const float roll_max_rad   = config.GetGyroSAFERoll()  * (float)M_PI / 180.0f;
    const float pitch_max_rad  = config.GetGyroSAFEPitch() * (float)M_PI / 180.0f;
    const float hyst_rad       = 1.0f * (float)M_PI / 180.0f; // 1° hysteresis to avoid chatter
    const float smooth_margin  = 6.0f * (float)M_PI / 180.0f; // start smoothing within 6° of the limit

    auto inside_with_hyst = [&](float a, float lim)->bool {
        // inside if comfortably within limit - hysteresis
        return fabsf(a) <= (lim - hyst_rad);
    };
    auto at_or_beyond = [&](float a, float lim)->bool {
        return fabsf(a) >= lim;
    };

    // === 3) Boundary PID (your original logic) ===
    auto calc_guard = [](PID& pid, float angle_rad, float max_rad) {
        if (fabsf(angle_rad) < max_rad) {
            pid.reset();
            pid.output = 0.0f;
        } else {
            const float setpoint = (angle_rad > 0.0f) ? +max_rad : -max_rad;
            pid.calculate(setpoint, angle_rad);
        }
    };
    calc_guard(pid_roll,  roll_rad,  roll_max_rad);
    calc_guard(pid_pitch, pitch_rad, pitch_max_rad);

    // === 4) Ignore outward stick past the limit (inward always allowed) ===
    const float stick_roll  = get_command(GYRO_AXIS_ROLL);   // [-1, +1]
    const float stick_pitch = get_command(GYRO_AXIS_PITCH);

    auto outward = [](float angle, float stick)->bool {
        return (angle >= 0.0f && stick > 0.0f) || (angle < 0.0f && stick < 0.0f);
    };
    auto stick_if_allowed = [&](float angle_rad, float max_rad, float stick)->float {
        if (at_or_beyond(angle_rad, max_rad) && outward(angle_rad, stick)) {
            return 0.0f; // hard block outward stick at/over limit
        }
        return stick;    // pass through otherwise
    };
    float stick_roll_eff  = stick_if_allowed(roll_rad,  roll_max_rad,  stick_roll);
    float stick_pitch_eff = stick_if_allowed(pitch_rad, pitch_max_rad, stick_pitch);

    // === 5) Compose raw outputs before smoothing ===
    float out_roll_raw  = stick_roll_eff  + pid_roll.output;
    float out_pitch_raw = stick_pitch_eff + pid_pitch.output;

    // === 6) Edge smoothing: only near/over the limit ===
    // We apply a first-order low-pass AND a slew limiter as we approach/exceed the limit.
    // Far from the limits, commands pass through unfiltered.
    auto smooth_factor = [&](float angle_rad, float lim_rad)->float {
        // 0 -> far inside, 1 -> at/beyond limit
        const float dist = lim_rad - fabsf(angle_rad);
        if (dist >= smooth_margin) return 0.0f;     // far: no smoothing
        if (dist <= 0.0f)         return 1.0f;     // over: max smoothing
        const float x = 1.0f - (dist / smooth_margin); // 0→1 as we approach limit
        // Smoothstep for gentle onset
        return x * x * (3.0f - 2.0f * x);
    };

    const float s_roll  = smooth_factor(roll_rad,  roll_max_rad);
    const float s_pitch = smooth_factor(pitch_rad, pitch_max_rad);

    // Low-pass: blend between passthrough (alpha=1) and strong filtering (alpha small)
    auto filtered = [&](float prev, float target, float s)->float {
        // Map s in [0,1] to a time-constant: near limit => bigger tau (stronger filter)
        const float tau_far  = 0.015f;  // ~15ms when far from limit (almost passthrough)
        const float tau_near = 0.090f;  // ~90ms near/over limit (smooth corrections)
        const float tau = tau_far * (1.0f - s) + tau_near * s;
        const float alpha = (tau <= 1e-6f) ? 1.0f : (dt / (tau + dt));
        return prev + alpha * (target - prev);
    };

    float out_roll_filt  = filtered(roll_cmd_slew,  out_roll_raw,  s_roll);
    float out_pitch_filt = filtered(pitch_cmd_slew, out_pitch_raw, s_pitch);

    // Slew limiter: cap change rate to avoid snaps when wind toggles around the edge
    auto slew_limit = [&](float prev, float target)->float {
        const float max_fullscale_per_s = 3.0f;         // up to 3.0 FS/sec (tweak to taste)
        const float max_step = max_fullscale_per_s * dt;
        const float delta = target - prev;
        if (delta >  max_step) return prev + max_step;
        if (delta < -max_step) return prev - max_step;
        return target;
    };

    roll_cmd_slew  = slew_limit(roll_cmd_slew,  out_roll_filt);
    pitch_cmd_slew = slew_limit(pitch_cmd_slew, out_pitch_filt);

    // === 7) Outputs ===
    // Roll/Pitch: Raw so we enforce the clamp & smoothing ourselves.
    setOutputRaw(GYRO_AXIS_ROLL,  roll_cmd_slew);
    setOutputRaw(GYRO_AXIS_PITCH, pitch_cmd_slew);

    // Yaw: rate damper as before; setOutput adds pilot yaw internally.
    pid_yaw.calculate(0.0f, -gyro.f_gyro[GYRO_AXIS_YAW]);
    setOutput(GYRO_AXIS_YAW, pid_yaw.output);
}

#endif
