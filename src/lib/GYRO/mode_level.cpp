#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "mode_level.h"
#include "FusionMath.h"

void LevelController::initialize()
{
    configure_pids(1.0, 1.0, 1.0);
}

void LevelController::update()
{
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
    setOutputRaw(GYRO_AXIS_ROLL, pid_roll.output);

    pid_pitch.calculate(0.0f, e_b.array[1]);
    setOutputRaw(GYRO_AXIS_PITCH, pid_pitch.output);

    // 7) Yaw rate damper (unchanged) + pass-through of stick in setOutput
    pid_yaw.calculate(0.0f, -gyro.f_gyro[GYRO_AXIS_YAW]);
    setOutputRaw(GYRO_AXIS_YAW, pid_yaw.output + yaw_cmd);
}

#endif
