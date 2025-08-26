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

static MPU6050 mpu;

bool GyroDevMPU6050::initialize()
{
    return mpu.testConnection();
}

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
static void dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
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

        gyro.f_gyro[GYRO_AXIS_PITCH] = v_gyro.x * gscale; // Pitch rate (radians/s)
        gyro.f_gyro[GYRO_AXIS_ROLL] = v_gyro.y * gscale; // Roll rate (radians/s)
        gyro.f_gyro[GYRO_AXIS_YAW] = v_gyro.z * gscale; // Yaw rate (radians/s)

        // Convert MPU6050 quaternion to Fusion quaternion format
        FusionQuaternion fusionQuat;
        fusionQuat.element.w = q.w;
        fusionQuat.element.x = q.y;
        fusionQuat.element.y = q.x;
        fusionQuat.element.z = q.z;

        // Update gyro quaternion
        gyro.quaternion = fusionQuat;

        // Create Fusion vectors for AHRS update
        FusionVector fusionGyro = {.axis = {gyro.f_gyro[GYRO_AXIS_ROLL], gyro.f_gyro[GYRO_AXIS_PITCH], gyro.f_gyro[GYRO_AXIS_YAW]}};

        // Note: We would need accelerometer data for full AHRS update, but for now use DMP quaternion directly
        // In a full implementation, you would call: FusionAhrsUpdate(&gyro.ahrs, fusionGyro, fusionAccel, fusionMag, deltaTime);
        // and then get quaternion from: gyro.quaternion = FusionAhrsGetQuaternion(&gyro.ahrs);
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
