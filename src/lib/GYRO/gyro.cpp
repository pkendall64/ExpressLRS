#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
#include "config.h"
#include "device.h"
#include "elrs_eeprom.h" // only needed to satisfy PIO
#include "gyro.h"
#include "gyro_mixer.h"
#include "gyro_mpu6050.h"
#include "gyro_types.h"
#include "logging.h"
#include "mode_hover.h"
#include "mode_launch.h"
#include "mode_level.h"
#include "mode_none.h"
#include "mode_rate.h"
#include "mode_safe.h"
#include "telemetry.h"

extern Telemetry telemetry;

PID pid_roll(1, -1, 0.0, 0.0, 0.0);
PID pid_pitch(1, -1, 0.0, 0.0, 0.0);
PID pid_yaw(1, -1, 0.0, 0.0, 0.0);

Controller* controllers[] = {
    new NoneController(),
    new RateController(),
    new SafeController(),
    new LevelController(),
    new LaunchController(),
    new HoverController()
};

Controller *controller = controllers[0];

#ifdef GYRO_BOOT_JITTER
static uint8_t boot_jitter_times = 0;
static uint32_t boot_jitter_time = 0;
static int8_t boot_jitter_offset = GYRO_BOOT_JITTER_US;

bool boot_jitter(uint16_t *us)
{
    if (boot_jitter_times > GYRO_BOOT_JITTER_TIMES)
        return false;

    if (millis() - boot_jitter_time > GYRO_BOOT_JITTER_MS)
    {
        boot_jitter_times++;
        boot_jitter_time = millis();
        boot_jitter_offset *= -1;
    }

    *us = *us + boot_jitter_offset;
    return true;
}
#endif

volatile gyro_event_t gyro_event = GYRO_EVENT_NONE;

void Gyro::calibrate()
{
    dev->calibrate();
    #ifdef GYRO_BOOT_JITTER
    boot_jitter_times = 0;
    boot_jitter_time = 0;
    #endif
}

void Gyro::detect_mode(uint16_t us)
{
    const rx_config_gyro_mode_pos_t *modes = config.GetGyroModePos();
    gyro_mode_t selected_mode;
    const uint8_t switch_pos = CRSF_to_SWITCH3b(us);
    switch (switch_pos)
    {
        case 0: selected_mode = (gyro_mode_t) modes->val.pos1; break;
        case 1: selected_mode = (gyro_mode_t) modes->val.pos2; break;
        case 2: selected_mode = (gyro_mode_t) modes->val.pos3; break;
        case 3: selected_mode = (gyro_mode_t) modes->val.pos4; break;
        case 4: selected_mode = (gyro_mode_t) modes->val.pos5; break;
        case 5: selected_mode = (gyro_mode_t) modes->val.pos6; break;
        case 7: selected_mode = (gyro_mode_t) modes->val.pos7; break;
        default: selected_mode = GYRO_MODE_OFF; break;
    }
    if (gyro_mode != selected_mode)
    {
        switch_mode(selected_mode);
    }
}

/**
 * Trigger a gyro re-initialization of the current gyro mode
 */
void Gyro::reload()
{
    switch_mode(GYRO_MODE_OFF);
}

void Gyro::switch_mode(const gyro_mode_t mode)
{
    DBGLN("Gyro: Switching mode to %d", mode);
    gyro_mode = mode;
    controller = controllers[gyro_mode];
    controller->configure();
}

void Gyro::detect_gain(const uint16_t us)
{
    gain = (us - GYRO_US_MIN) / (GYRO_US_MAX - GYRO_US_MIN) * 500.0f;
}

/**
 * Apply gyro servo output mixing and detect gyro mode
 */
void Gyro::mixer(const uint8_t ch, uint16_t *us)
{
    // We get called before the gyro configuration is initialized
    if (!initialized) return;

    mixer_channel_update(ch, *us);

    #ifdef GYRO_BOOT_JITTER
    boot_jitter(us);
    #endif
}

void Gyro::send_telemetry()
{
    CRSF_MK_FRAME_T(crsf_sensor_attitude_t)
    crsfAttitude = {};

    // Scale radians to 100µ-radians for CRSF protocol
    crsfAttitude.p.pitch = htobe16((int16_t)(gyro.ypr[1] * 10000.0f));
    crsfAttitude.p.roll = htobe16((int16_t)(gyro.ypr[2] * 10000.0f));
    crsfAttitude.p.yaw = htobe16((int16_t)(gyro.ypr[0] * 10000.0f));

    CRSF::SetHeaderAndCrc((uint8_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
    telemetry.AppendTelemetryPackage((uint8_t *)&crsfAttitude);
}

bool Gyro::read_device()
{
    if (dev->read())
    {
        // Calculate gyro update rate in HZ
        update_rate = 1.0 / ((micros() - last_update) / 1000000.0);
        last_update = micros();
        send_telemetry();
    }
    return DURATION_IMMEDIATELY;
}

// #define GYRO_PID_DEBUG_TIME 100
unsigned long gyro_debug_time = 0;

#ifdef GYRO_PID_DEBUG_TIME
void _make_gyro_debug_string(PID *pid, char *str)
{
    sprintf(str, "Setpoint: %5.2f PV: %5.2f I:%5.2f D:%5.2f Error: %5.2f Out: %5.2f",
        pid->setpoint, pid->pv, pid->Iout, pid->Dout, pid->error, pid->output);
}
#endif


void Gyro::tick()
{
    if (!initialized) return;

    if (boot_jitter_times < GYRO_BOOT_JITTER_TIMES) return;

    if ((micros() - pid_delay) < 1000 ) return; // ~1k PID loop
    pid_delay = micros();

    detect_mode(ChannelMixedData[MIX_DESTINATION_GYRO_MODE]);
    // convert the gain from -1 to +1 float to 0 - 1.0
    gain = (1 + CRSF_to_FLOAT((uint16_t) ChannelMixedData[MIX_DESTINATION_GYRO_GAIN])) / 2;

    controller->update();

    #ifdef GYRO_PID_DEBUG_TIME
    if (gyro_mode != GYRO_MODE_OFF && micros() - gyro_debug_time > GYRO_PID_DEBUG_TIME * 1000 )
    {
        char piddebug[128];
        _make_gyro_debug_string(&pid_pitch, piddebug);
        DBGLN("\nPID Pitch %s", piddebug);
        _make_gyro_debug_string(&pid_roll, piddebug);
        DBGLN("PID Roll  %s", piddebug);
        _make_gyro_debug_string(&pid_yaw, piddebug);
        DBGLN("PID Yaw   %s", piddebug);
        DBGLN("GAIN %f", gain);
        gyro_debug_time = micros();
    }
    #endif
}

#endif
