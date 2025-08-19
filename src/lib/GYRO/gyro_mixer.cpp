#if defined(PLATFORM_ESP32) && defined(TARGET_RX)
#include "targets.h"
#include "gyro.h"
#include "gyro_types.h"
#include "logging.h"

#define GYRO_SUBTRIM_INIT_SAMPLES 50
uint8_t subtrim_init = 0;

// Channel configuration for minimum, subtrim and maximum us values. If no
// values are specified, we default to full range and autodetection of midpoint.
uint16_t midpoint[GYRO_MAX_CHANNELS] = {};

bool ch_map_auto_subtrim[GYRO_MAX_CHANNELS] = {};
bool auto_subtrim_complete = false;

unsigned long time_of_first_packet = 0;

void mixer_initialize()
{
    // Called once during boot to fill arrays and sanity check
    for (int i = 0; i < GYRO_MAX_CHANNELS; i++)
    {
        // Note that if we have no channel configuration, all values start at
        // zero, in that case apply defaults.
        if (midpoint[i] == 0) {
            midpoint[i] = GYRO_US_MID;
            ch_map_auto_subtrim[i] = true;
        }
    }
}

static void auto_subtrim(uint8_t ch, uint16_t us)
{
    // Set midpoint (subtrim) from an average of a set of samples
    if (ch_map_auto_subtrim[ch] && subtrim_init < GYRO_SUBTRIM_INIT_SAMPLES) {
        midpoint[ch] = (midpoint[ch] + us) / 2;
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

#endif
