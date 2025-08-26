#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_level.h"
#include "FusionMath.h"

void LevelController::initialize() {
    configure_pids(1.0, 1.0, 1.0);
    prev_us = 0;
    roll_smooth.reset();
    pitch_smooth.reset();
}

void LevelController::update()
{
    const uint32_t now_us = micros();
    const float dt = (prev_us == 0) ? 0.002f : fmaxf(1e-3f, (now_us - prev_us) * 1e-6f);
    prev_us = now_us;

    const float roll_cmd  = get_command(GYRO_AXIS_ROLL);
    const float pitch_cmd = get_command(GYRO_AXIS_PITCH);
    const float yaw_cmd   = get_command(GYRO_AXIS_YAW);

    const float roll_max_rad  = (float)config.GetGyroLevelRoll()  * (float)M_PI / 180.0f;
    const float pitch_max_rad = (float)config.GetGyroLevelPitch() * (float)M_PI / 180.0f;

    // decoupled body-frame mapping with optional pitch bias (set by Launch)
    const float alpha = roll_cmd  * roll_max_rad;
    float       beta  = -pitch_cmd * pitch_max_rad - pitch_bias;
    beta = fmaxf(-pitch_max_rad, fminf(pitch_max_rad, beta));

    // desired body "down" (normalized)
    const float u = tanf(alpha), v = tanf(beta);
    FusionVector z_des_b = {{ -v, u, 1.0f }};
    z_des_b = FusionVectorNormalise(z_des_b);

    // measured body "down" from quaternion
    const FusionVector z_meas_b = quatBodyDown(gyro.quaternion);

    // error: e_b = z_meas_b × z_des_b  (use x,y for roll/pitch)
    const FusionVector e_b = FusionVectorCrossProduct(z_meas_b, z_des_b);

    pid_roll.calculate(0.0f,  e_b.array[0]);
    pid_pitch.calculate(0.0f, e_b.array[1]);

    // scale stabilization by gyro.gain and smooth near the limits
    float roll_rad_meas, pitch_rad_meas;
    quatToRollPitch(gyro.quaternion, roll_rad_meas, pitch_rad_meas);

    const float out_roll_raw  = gyro.gain * pid_roll.output;
    const float out_pitch_raw = gyro.gain * pid_pitch.output;

    const float out_roll  = roll_smooth.process (roll_rad_meas,  roll_max_rad,  out_roll_raw,  dt);
    const float out_pitch = pitch_smooth.process(pitch_rad_meas, pitch_max_rad, out_pitch_raw, dt);

    // Outputs: Raw on roll/pitch to avoid auto RC add; yaw = Raw + manual RC add
    setOutputRaw(GYRO_AXIS_ROLL,  out_roll);
    setOutputRaw(GYRO_AXIS_PITCH, out_pitch);

    pid_yaw.calculate(0.0f, -gyro.f_gyro[GYRO_AXIS_YAW]);
    setOutputRaw(GYRO_AXIS_YAW,  gyro.gain * pid_yaw.output + yaw_cmd);
}

#endif
