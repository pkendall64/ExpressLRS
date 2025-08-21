#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
#include "config.h"
#include "gyro.h"
#include "gyro_mpu6050.h"
#include "logging.h"

#include "MPU6050_6Axis_MotionApps612.h"

#define gscale ((250. / 32768.0) / 100) // gyro default 250 LSB per d/s

// MPU control/status vars
static uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
static uint16_t fifoCount;       // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64];   // FIFO storage buffer

#ifdef DEBUG_GYRO_STATS
/**
 * For debugging print useful gyro state
 */
void GyroDevMPU6050::print_gyro_stats()
{
    if (millis() - last_gyro_stats_time < 500)
        return;

    // Calculate gyro update rate in HZ
    int update_rate = 1.0 /
                      ((micros() - gyro.last_update) / 1000000.0);

    char rate_str[5]; sprintf(rate_str, "%4d", update_rate);

    char roll_str[8]; sprintf(roll_str, "%6.2f", gyro.f_angle[GYRO_AXIS_ROLL] * 180 / M_PI);
    char pitch_str[8]; sprintf(pitch_str, "%6.2f", gyro.f_angle[GYRO_AXIS_PITCH] * 180 / M_PI);
    char yaw_str[8]; sprintf(yaw_str, "%6.2f", gyro.f_angle[GYRO_AXIS_YAW] * 180 / M_PI);

    char gyro_x[8]; sprintf(gyro_x, "%6.2f", gyro.f_gyro[GYRO_AXIS_ROLL]);
    char gyro_y[8]; sprintf(gyro_y, "%6.2f", gyro.f_gyro[GYRO_AXIS_PITCH]);
    char gyro_z[8]; sprintf(gyro_z, "%6.2f", gyro.f_gyro[GYRO_AXIS_YAW]);

    DBGLN(
        "%s HZ "
        "Gain %f "
        "Pitch: %s Roll: %s Yaw: %s "
        "Gyro x: %s Gyro y: %s Gyro z: %s "
        ,rate_str
        ,gyro.gain
        ,pitch_str, roll_str, yaw_str
        ,gyro_x, gyro_y, gyro_z
        );

    last_gyro_stats_time = millis();
}
#endif

static MPU6050 mpu;

void GyroDevMPU6050::calibrate()
{
    mpu.reset();
    vTaskDelay(50 * portTICK_PERIOD_MS);

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setMasterClockSpeed(13); // 400kHz
    mpu.setRate(0);              // Max rate?

    // INT_PIN_CFG
    mpu.setInterruptMode(0);         // INT_LEVEL_HIGH
    mpu.setInterruptDrive(0);        // INT_OPEN_DIS (push-pull)
    mpu.setInterruptLatch(0);        // LATCH_INT_DIS (50us)
    mpu.setInterruptLatchClear(0);   // INT_RD_CLEAR_DIS (read-only)
    mpu.setFSyncInterruptLevel(0);   // FSYNC_INT_LEVEL_HIGH (active-high)
    mpu.setFSyncInterruptEnabled(0); // FSYNC_INT_DIS
    mpu.setI2CBypassEnabled(1);      // I2C_BYPASS_EN
    mpu.setClockOutputEnabled(0);    // CLOCK_DIS
    mpu.setExternalFrameSync(0);

    mpu.dmpInitialize();

    // Run the calibration
    mpu.CalibrateAccel(8);
    mpu.CalibrateGyro(8);

    // Store the calibration offsets
    config.SetAccelCalibration(
        mpu.getXAccelOffset(),
        mpu.getYAccelOffset(),
        mpu.getZAccelOffset()
    );
    config.SetGyroCalibration(
        mpu.getXGyroOffset(),
        mpu.getYGyroOffset(),
        mpu.getZGyroOffset()
    );

    start(false);
}

uint8_t GyroDevMPU6050::start(bool calibrate) {
    DBGLN("Initialize_mpu");
    mpu.reset();
    vTaskDelay(50 * portTICK_PERIOD_MS);

    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setMasterClockSpeed(13); // 400kHz
    mpu.setRate(0);              // Max rate?

    // INT_PIN_CFG
    mpu.setInterruptMode(0);         // INT_LEVEL_HIGH
    mpu.setInterruptDrive(0);        // INT_OPEN_DIS (push-pull)
    mpu.setInterruptLatch(0);        // LATCH_INT_DIS (50us)
    mpu.setInterruptLatchClear(0);   // INT_RD_CLEAR_DIS (read-only)
    mpu.setFSyncInterruptLevel(0);   // FSYNC_INT_LEVEL_HIGH (active-high)
    mpu.setFSyncInterruptEnabled(0); // FSYNC_INT_DIS
    mpu.setI2CBypassEnabled(1);      // I2C_BYPASS_EN
    mpu.setClockOutputEnabled(0);    // CLOCK_DIS

    mpu.setExternalFrameSync(0);
    mpu.dmpInitialize();

    const rx_config_gyro_calibration_t *offsets;
    offsets = config.GetAccelCalibration();
    mpu.setXAccelOffset(offsets->x);
    mpu.setYAccelOffset(offsets->y);
    mpu.setZAccelOffset(offsets->z);
    offsets = config.GetGyroCalibration();
    mpu.setXGyroOffset(offsets->x);
    mpu.setYGyroOffset(offsets->y);
    mpu.setZGyroOffset(offsets->z);

    mpu.setDMPEnabled(true);

    return DURATION_IMMEDIATELY; // Call timeout() immediately
}

/**
 * This method is used instead of mpu.dmpGetYawPitchRoll() as that method has
 * issues when gravity switches at high pitch angles.
*/
void GyroDevMPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
    // roll: (tilt left/right, about X axis)
    data[GYRO_AXIS_ROLL] = atan2(gravity -> y , gravity -> z);
    // pitch: (nose up/down, about Y axis)
    data[GYRO_AXIS_PITCH] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // yaw: (about Z axis)
    data[GYRO_AXIS_YAW] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);

    // NOTE: This is buggy at high pitch angles when gravity flips
    // if (gravity -> z < 0) {
    //     if(data[1] > 0) {
    //         data[1] = PI - data[1];
    //     } else {
    //         data[1] = -PI - data[1];
    //     }
    // }
}

bool GyroDevMPU6050::read() {
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        DBGLN("Resetting gyro FIFO buffer");
        mpu.resetFIFO();
        return false;
    }
    if (mpuIntStatus & 0x02)
    {
        int result = mpu.GetCurrentFIFOPacket(fifoBuffer, 28);
        if (result != 1)
            return DURATION_IMMEDIATELY;

        VectorInt16 v_gyro;
        Quaternion q;       // [w, x, y, z]         quaternion container
        mpu.dmpGetGyro(&v_gyro, fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer); // [w, x, y, z] quaternion container

        const gyro_sensor_align_t alignment = config.GetGyroSensorAlignment();
        if (alignment != GYRO_ALIGN_CW0_DEG)
        {
            Quaternion v_rotation(0, 0, 0, 0);
            Quaternion q_rotation(0, 0, 0, 0);

            switch (alignment)
            {
            case GYRO_ALIGN_CW90_DEG:
                q_rotation.w = 0.7071;
                q_rotation.z = 0.7071;
                v_rotation.w = 0.7071;
                v_rotation.z = 0.7071;
                break;

            case GYRO_ALIGN_CW180_DEG:
                q_rotation.z = 1;
                v_rotation.z = 1;
                break;

            case GYRO_ALIGN_CW270_DEG:
                q_rotation.w = 0.7071;
                q_rotation.z = -0.7071;
                v_rotation.w = 0.7071;
                v_rotation.z = -0.7071;
                break;

            case GYRO_ALIGN_CW0_DEG_FLIP:
                v_rotation.x = 1;
                q_rotation.x = 1;
                break;

            case GYRO_ALIGN_CW180_DEG_FLIP:
                // TODO
            case GYRO_ALIGN_CW90_DEG_FLIP:
                // TODO
            case GYRO_ALIGN_CW270_DEG_FLIP:
                // TODO
            default: ;
            }

            v_gyro = v_gyro.getRotated(&v_rotation);
            q = q.getProduct(q_rotation);
        }

        gyro.f_gyro[GYRO_AXIS_ROLL] = v_gyro.x * gscale; // Roll rate (radians/s)
        gyro.f_gyro[GYRO_AXIS_PITCH] = v_gyro.y * gscale; // Pitch rate (radians/s)
        gyro.f_gyro[GYRO_AXIS_YAW] = v_gyro.z * gscale; // Yaw rate (radians/s)
        VectorFloat gravity;
        mpu.dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(gyro.f_angle, &q, &gravity);
    }
    else
    {
        return false;
    }

    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats();
    #endif

    return true;
}
#endif
