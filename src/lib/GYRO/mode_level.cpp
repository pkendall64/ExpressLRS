#if defined(PLATFORM_ESP32) && defined(TARGET_RX)

#include "crsf_protocol.h"
#include "gyro.h"
#include "gyro_mixer.h"

/**
 * Airplane Level/Stable Mode
 *
 * This mode tries to keep the plane flying level (pitch/roll) when there is no
 * stick input.
 */

float pitch_offset = 0.0;

#define degToRad(angleInDegrees) (float)((angleInDegrees) * M_PI / 180.0)

void level_controller_initialize(const float offset)
{
    pitch_offset = offset;
    configure_pids(1.0, 1.0, 1.0);
}

extern uint16_t midpoint[GYRO_MAX_CHANNELS];
extern bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS];

/**
 * Get a -1 to +1 float for the gyro input command
 */
float get_command(mix_destination_t type)
{
    for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
    {
        const rx_config_mix_t *mix = config.GetMix(mix_number);
        if (!mix->val.active || mix->val.destination != type)
            continue;

        const uint8_t ch = mix->val.source;
        const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;
        const uint16_t val_us = CRSF_to_US(ChannelData[mix->val.source]) - (mid - GYRO_US_MID);
        return us_command_to_float(val_us);
    }
    return 0;
}

void level_controller_calculate_pid()
{
    pid_roll.calculate(
        get_command(MIX_DESTINATION_GYRO_ROLL) * degToRad(config.GetGyroLevelRoll()),
        gyro.ypr[2]
    );

    pid_pitch.calculate(
        get_command(MIX_DESTINATION_GYRO_PITCH), // * degToRad(config.GetGyroLevelPitch()),
        // For the pitch axis in launch mode (pitch_offset != 0)
        // we change what the PID controller sees as level
        degToRad(pitch_offset) - gyro.ypr[1]
    );
    
    pid_yaw.calculate(0, -gyro.f_gyro[2]);
    // pid_yaw.output += command;
}
#endif
