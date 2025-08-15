#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
#include "targets.h"
#include "config.h"
#include "mixer.h"
#include "gyro.h"
#include "logging.h"
#include "crsf_protocol.h"

#define GYRO_SUBTRIM_INIT_SAMPLES 50
uint8_t subtrim_init = 0;

// Channel configuration for minimum, subtrim and maximum us values. If no
// values are specified we default to full range and autodetection of midpoint.
uint16_t midpoint[GYRO_MAX_CHANNELS] = {};

bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS] = {};
bool auto_subtrim_complete = false;

unsigned long time_of_first_packet = 0;

void mixer_initialize()
{
    // Called once during boot to fill arrays and sanity check
    for (int i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        // Note that if we have no channel configuration all values start at
        // zero, in that case apply defaults.
        if (midpoint[i] == 0) {
            midpoint[i] = GYRO_US_MID;
            ch_map_auto_subtrim[i] = true;
        }
    }
}

void auto_subtrim(uint8_t ch, uint16_t us)
{
    // Set midpoint (subtrim) from an average of a set of samples
    if (ch_map_auto_subtrim[ch] && subtrim_init < GYRO_SUBTRIM_INIT_SAMPLES) {
        midpoint[ch] = (midpoint[ch] + us) / 2;
        //midpoint[ch] = (((midpoint[ch] * subtrim_init) / subtrim_init) + us) / 2;
    }
}

void mixer_channel_update(uint8_t ch, uint16_t us)
{
    if (time_of_first_packet == 0) {
        time_of_first_packet = millis();
        return;
    }
    // If we don't wait the subtrims are incorrect... hmm
    if (millis() - time_of_first_packet < 1000)
        return;

    if (!auto_subtrim_complete) {
        if (ch == 0 && ++subtrim_init > GYRO_SUBTRIM_INIT_SAMPLES) {
            auto_subtrim_complete = true;
            for (unsigned i = 0; i < GYRO_MAX_CHANNELS; i++) {
                DBGLN("Subtrim channel %d: %d", i, midpoint[i]);
            }
        } else {
            auto_subtrim(ch, us);
        }
    }
}

/**
 *  Apply a correction to a servo PWM value
 */
float us_command_to_float(uint16_t us)
{
    // TODO: this will take into account subtrim and max throws
    return us <= GYRO_US_MID
        ? float(us - GYRO_US_MID) / (GYRO_US_MID - GYRO_US_MIN)
        : float(us - GYRO_US_MID) / (GYRO_US_MAX - GYRO_US_MID);
}

/**
 * Convert a CRSF value to a float
 */
float crsf_command_to_float(uint16_t command)
{
    return command <= CRSF_CHANNEL_VALUE_MID
        ? float (command - CRSF_CHANNEL_VALUE_MID) / (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN)
        : float (command - CRSF_CHANNEL_VALUE_MID) / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID);
}

/**
 * Convert a channel µs value to a float command
 *
 * This takes into account subtrim and max throws.
 */
float us_command_to_float(uint8_t ch, uint16_t us)
{
    // TODO: inverted matters?
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(ch);
    const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;
    return us <= mid
        ? float(us - mid) / (mid - limits->val.min)
        : float(us - mid) / (limits->val.max - mid);
}

/**
 * Convert +-1.0 float into µs for an output channel
 *
 * This takes into account subtrim and max throws.
 */
uint16_t float_to_us(uint8_t ch, float value)
{
    const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(ch);
    const uint16_t mid = ch_map_auto_subtrim[ch] ? midpoint[ch] : GYRO_US_MID;

    return value < 0
        ? mid + ((mid - limits->val.min) * value)
        : mid + ((limits->val.max - mid) * value);
}

void applyMixes()
{
    int32_t newChannelData[CRSF_NUM_CHANNELS + GYRO_DESTINATIONS];
    for (int i = 0; i < CRSF_NUM_CHANNELS + GYRO_DESTINATIONS; i++)
    {
        newChannelData[i] = CRSF_CHANNEL_VALUE_MID;
    }

    for (unsigned mix_number = 0; mix_number < MAX_MIXES; mix_number++)
    {
        const rx_config_mix_t *mix = config.GetMix(mix_number);

        if (!mix->val.active)
            continue;

        newChannelData[mix->val.destination] += mix->val.offset;

        // The fist 16 enums are CRSF input channels, and the next three are gyro outputs
        if (mix->val.source < CRSF_NUM_CHANNELS + GYRO_SOURCES)
        {
            const auto crsfVal = (int32_t)ChannelData[mix->val.source] - CRSF_CHANNEL_VALUE_MID;
            const auto scale = crsfVal < 0 ? (int)mix->val.weight_negative : (int)mix->val.weight_positive;
            const auto scaled = crsfVal * scale / 100;
            newChannelData[mix->val.destination] += scaled;
        }
        else
        {
            switch ((mix_source_t) mix->val.source)
            {
            case MIX_SOURCE_FAILSAFE: {
                // This is a full max throw input when we are in failsafe which
                // can be mixed to other channels.
                const auto scale = (int) mix->val.weight_positive;
                const auto result = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID) * scale / 100;
                newChannelData[mix->val.destination] += result;
                break;
            }

                // Later we may add other source mixes here (gyro, failsafe, etc)
            default:
                break;
            }
        }
    }

    for (unsigned ch = 0; ch < CRSF_NUM_CHANNELS + GYRO_DESTINATIONS; ch++)
    {
        ChannelMixedData[ch] = constrain(newChannelData[ch], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    }
}

#endif
