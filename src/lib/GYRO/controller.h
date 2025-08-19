#pragma once
#include "crsf_protocol.h"
#include "gyro.h"
#include "gyro_mixer.h"
#include "logging.h"

extern uint16_t midpoint[GYRO_MAX_CHANNELS];
extern bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS];

class Controller
{
public:
    virtual ~Controller() = default;

    virtual void initialize() = 0;
    virtual void update() = 0;

protected:
    /**
     * Get a -1 to +1 float for the gyro input command
     */
    static float get_command(const mix_destination_t type)
    {
        for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
        {
            const rx_config_mix_t *mix = config.GetMix(mix_number);
            if (mix->val.active && mix->val.destination == type)
            {
                const uint8_t ch = mix->val.source;
                const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;
                const uint16_t us = CRSF_to_US(ChannelData[mix->val.source]) - (mid - GYRO_US_MID);
                return us <= GYRO_US_MID
                    ? float(us - GYRO_US_MID) / (GYRO_US_MID - GYRO_US_MIN)
                    : float(us - GYRO_US_MID) / (GYRO_US_MAX - GYRO_US_MID);
            }
        }
        return 0.0f;
    }

    static float getChannelData(const mix_destination_t destination)
    {
        for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
        {
            const rx_config_mix_t *mix = config.GetMix(mix_number);
            if (mix->val.active && mix->val.destination == destination)
            {
                return CRSF_to_FLOAT((uint16_t) ChannelMixedData[mix->val.source]);
            }
        }
        return 0.0f;
    }

    static void configure_pid_gains(PID * const pid, const rx_config_gyro_gains_t *gains, const float max, const float min)
    {
        DBGLN("Config gains: P %d I %d D %d G %d", gains->p, gains->i, gains->d, gains->gain);
        if (max == 0.0 && min == 0.0) {
            // No gyro correction on this axis
            pid->configure(0.0, 0.0, 0.0, 0.0, 0.0);
        } else {
            const float p = gains->gain * gains->p / 1000.0;
            const float i = gains->gain * gains->i / 1000.0;
            const float d = gains->gain * gains->d / 1000.0;
            DBGLN("PID gains: P %f I %f D %f", p, i, d);

            pid->configure(p, i, d, max, min);
        }
        pid->reset();
    }

    static void configure_pids(const float roll_limit, const float pitch_limit, const float yaw_limit)
    {
        const rx_config_gyro_gains_t *roll_gains = config.GetGyroGains(GYRO_AXIS_ROLL);
        const rx_config_gyro_gains_t *pitch_gains = config.GetGyroGains(GYRO_AXIS_PITCH);
        const rx_config_gyro_gains_t *yaw_gains = config.GetGyroGains(GYRO_AXIS_YAW);

        configure_pid_gains(&pid_roll, roll_gains, roll_limit, -1.0 * roll_limit);
        configure_pid_gains(&pid_pitch, pitch_gains, pitch_limit, -1.0 * pitch_limit);
        configure_pid_gains(&pid_yaw, yaw_gains, yaw_limit, -1.0 * yaw_limit);
    }
};