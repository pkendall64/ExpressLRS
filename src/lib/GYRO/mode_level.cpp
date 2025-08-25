#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_level.h"
#include "FusionMath.h"

void LevelController::initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void LevelController::update()
{
    const uint32_t now_us = micros();
    const float dt = (prev_us == 0) ? 0.002f : fmaxf(1e-3f, (now_us - prev_us) * 1e-6f);
    prev_us = now_us;

    // 0) Pilot commands in [-1, 1]
    const float roll_cmd  = get_command(GYRO_AXIS_ROLL);
    const float pitch_cmd = get_command(GYRO_AXIS_PITCH);
    const float yaw_cmd = get_command(GYRO_AXIS_YAW);

    // 1) User angle limits (deg -> rad) via FusionMath
    const float roll_max_rad  = FusionDegreesToRadians(config.GetGyroLevelRoll());
    const float pitch_max_rad = FusionDegreesToRadians(config.GetGyroLevelPitch());

    // 2) Map sticks to body-frame tilt components (decoupled)
    const float alpha = roll_cmd  * roll_max_rad;   // desired roll
    const float beta  = fminf(pitch_max_rad, fmaxf(-pitch_max_rad, pitch_cmd * pitch_max_rad - pitch_bias));    // desired pitch
    const float u = tanf(alpha);  // roll component (body Y)
    const float v = tanf(beta);   // pitch component (body X)

    // 3) Desired down vector in BODY frame: z_des_b ∝ [-v, u, 1], then normalise
    FusionVector z_des_b = {{ -v, u, 1.0f }};
    z_des_b = FusionVectorNormalise(z_des_b);

    // 4) Get measured down vector in BODY frame using quaternion utilities
    const FusionQuaternion q_be = gyro.quaternion;             // your AHRS quat
    const FusionQuaternion q_eb = FusionQuaternionConjugate(q_be);
    constexpr FusionVector z_e = {{ 0.0f, 0.0f, 1.0f }};           // Earth “down”
    const FusionVector z_meas_b = rotateVecByQuat(q_eb, z_e);

    // 5) Body-frame tilt error: e_b = z_meas_b × z_des_b
    const FusionVector e_b = FusionVectorCrossProduct(z_meas_b, z_des_b);

    // 6) Drive roll/pitch PIDs to zero error
    pid_roll.calculate(0.0f,  e_b.array[0]);
    pid_pitch.calculate(0.0f, e_b.array[1]);

    // get actual roll/pitch angles to feed the same smoothers/limits
    float roll_rad_meas, pitch_rad_meas;
    quatToRollPitch(gyro.quaternion, roll_rad_meas, pitch_rad_meas);

    // Compose raw outputs (pilot stick already used in setpoint; keep it simple here)
    const float out_roll_raw  = pid_roll.output;
    const float out_pitch_raw = pid_pitch.output;

    // Smooth near Level limits using the same EdgeSmoother
    const float out_roll  = roll_smooth.process (roll_rad_meas,  roll_max_rad,  out_roll_raw,  dt);
    const float out_pitch = pitch_smooth.process(pitch_rad_meas, pitch_max_rad, out_pitch_raw, dt);

    // Outputs: stay Raw so we don’t double-add RC
    setOutputRaw(GYRO_AXIS_ROLL,  out_roll);
    setOutputRaw(GYRO_AXIS_PITCH, out_pitch);

    // yaw rate damper stays as before
    pid_yaw.calculate(0.0f, -gyro.f_gyro[GYRO_AXIS_YAW]);
    setOutputRaw(GYRO_AXIS_YAW, pid_yaw.output + yaw_cmd);
}

#endif
