#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "config.h"
#include "gyro.h"
#include "gyro_icm42688.h"
#include "logging.h"

static volatile bool interrupted = false;
static volatile unsigned long updated = 0;

static void IRAM_ATTR mpu_isr()
{
    updated = micros();
    interrupted = true;
}

bool GyroDevICM42688::initialize()
{
    if (mpu.begin() != 1)
        return false;

    // setting the accelerometer full scale range to +/-16G
    mpu.setAccelFS(ICM42688::gpm16);
    // setting the gyroscope full scale range to +/-250 deg/s
    mpu.setGyroFS(ICM42688::dps250);

    // set output data rate to 200 Hz
    mpu.setAccelODR(ICM42688::odr200);
    mpu.setGyroODR(ICM42688::odr200);

    pinMode(35, INPUT_PULLUP);
    attachInterrupt(35, mpu_isr, RISING);
    mpu.enableDataReadyInterrupt();

    FusionAhrsInitialise(&fusion);
    // Set AHRS algorithm settings
    constexpr FusionAhrsSettings settings = {
        .convention = FusionConventionEnu,
        .gain = 0.5f,
        .gyroscopeRange = 250, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5U * 250, /* 5 seconds */
    };
    FusionAhrsSetSettings(&fusion, &settings);
    return true;
}

void GyroDevICM42688::calibrate()
{
    mpu.calibrateGyro();
    config.SetGyroCalibration(
        (uint16_t)(mpu.getGyroBiasX() * 32768 / 2000),
        (uint16_t)(mpu.getGyroBiasY() * 32768 / 2000),
        (uint16_t)(mpu.getGyroBiasZ() * 32768 / 2000)
    );

    mpu.calibrateAccel();
    config.SetAccelCalibration(
        (uint16_t)(mpu.getAccelBiasX_mss() * 32768 / 16),
        (uint16_t)(mpu.getAccelBiasY_mss() * 32768 / 16),
        (uint16_t)(mpu.getAccelBiasZ_mss() * 32768 / 16)
    );
}

uint8_t GyroDevICM42688::start(bool calibrate) {
    DBGLN("Initialize_mpu");
    // mpu.begin();

    const rx_config_gyro_calibration_t *offsets;
    offsets = config.GetAccelCalibration();
    offsets = config.GetGyroCalibration();
    // mpu.setGyroBiasX(offsets->x / (2000.0 / 32768.0));
    // mpu.setGyroBiasY(offsets->y / (2000.0 / 32768.0));
    // mpu.setGyroBiasZ(offsets->z / (2000.0 / 32768.0));

    FusionAhrsReset(&fusion);
    return DURATION_IMMEDIATELY; // Call timeout() immediately
}

bool GyroDevICM42688::read() {
    const auto hasHardwarePin = true;//hardware_pin(HARDWARE_i2c_int) != UNDEF_PIN;
    const auto hasDataReady = mpu.isDataReady();
    const auto canReadData = interrupted || (hasHardwarePin && hasDataReady);
    if (!canReadData)
    {
        return false;
    }

    if (hasHardwarePin)
    {
        interrupted = false;
    }
    if (mpu.getAGT() < 0)
        return false;

    FusionVector v_gyro;
    v_gyro.axis.x = mpu.gyrX();
    v_gyro.axis.y = mpu.gyrY();
    v_gyro.axis.z = mpu.gyrZ();

    FusionVector v_accel;
    v_accel.axis.x = mpu.accX();
    v_accel.axis.y = mpu.accY();
    v_accel.axis.z = mpu.accZ();

    // const gyro_sensor_align_t alignment = config.GetGyroSensorAlignment();
    // if (alignment != GYRO_ALIGN_CW0_DEG)
    // {
    //     Quaternion v_rotation(0, 0, 0, 0);
    //     Quaternion q_rotation(0, 0, 0, 0);
    //
    //     switch (alignment)
    //     {
    //     case GYRO_ALIGN_CW90_DEG:
    //         q_rotation.w = 0.7071;
    //         q_rotation.z = 0.7071;
    //         v_rotation.w = 0.7071;
    //         v_rotation.z = 0.7071;
    //         break;
    //
    //     case GYRO_ALIGN_CW180_DEG:
    //         q_rotation.z = 1;
    //         v_rotation.z = 1;
    //         break;
    //
    //     case GYRO_ALIGN_CW270_DEG:
    //         q_rotation.w = 0.7071;
    //         q_rotation.z = -0.7071;
    //         v_rotation.w = 0.7071;
    //         v_rotation.z = -0.7071;
    //         break;
    //
    //     case GYRO_ALIGN_CW0_DEG_FLIP:
    //         v_rotation.x = 1;
    //         q_rotation.x = 1;
    //         break;
    //
    //     case GYRO_ALIGN_CW180_DEG_FLIP:
    //         // TODO
    //     case GYRO_ALIGN_CW90_DEG_FLIP:
    //         // TODO
    //     case GYRO_ALIGN_CW270_DEG_FLIP:
    //         // TODO
    //     default: ;
    //     }
    //
    //     v_gyro = v_gyro.getRotated(&v_rotation);
    //     v_accel = v_accel.getRotated(&v_rotation);
    // }

    gyro.f_gyro[GYRO_AXIS_ROLL] = v_gyro.axis.x; // Roll rate (radians/s)
    gyro.f_gyro[GYRO_AXIS_PITCH] = v_gyro.axis.y; // Pitch rate (radians/s)
    gyro.f_gyro[GYRO_AXIS_YAW] = v_gyro.axis.z; // Yaw rate (radians/s)

    gyro.f_accel[GYRO_AXIS_ROLL] = v_accel.axis.x; // Roll rate (radians/s)
    gyro.f_accel[GYRO_AXIS_PITCH] = v_accel.axis.y; // Pitch rate (radians/s)
    gyro.f_accel[GYRO_AXIS_YAW] = v_accel.axis.z; // Yaw rate (radians/s)

    // update filter
    const auto now = micros();//hasHardwarePin ? updated : micros();
    const auto dt = (now - last_update) / 1000000.0f;
    last_update = now;
    FusionAhrsUpdate(&fusion, v_gyro, v_accel, FUSION_VECTOR_ZERO, dt);

    // Update gyro quaternion directly from AHRS
    gyro.quaternion = FusionAhrsGetQuaternion(&fusion);

    // Note: Euler angles for telemetry are now computed on-demand in gyro.cpp send_telemetry()
    // This eliminates the custom quaternion-to-euler conversion and potential gimbal lock issues

    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats();
    #endif

    return true;
}
#endif
