#include "targets.h"

#if defined(PLATFORM_ESP32)
#include "config.h"
#include "crsf_protocol.h"
#include "devGyro.h"
#include "gyro.h"
#include "utils.h"

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
               ? float(command - CRSF_CHANNEL_VALUE_MID) / (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_STD_MIN)
               : float(command - CRSF_CHANNEL_VALUE_MID) / (CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_MID);
}

/**
 * Compose the gyro orientation labels from the hardware definition.
 * The hardware supplies just the descriptive part of each face label
 * (e.g. "UART Up;UART Dn;Pins Up;Pins Dn;Lbl Up;Lbl Dn"), the fixed
 * axis part (X+) is appended here. Faces without a description fall
 * back to the plain axis name.
 */
#ifndef GYRO_ORIENTATION_NAMES
#define GYRO_ORIENTATION_NAMES ((const char *)nullptr)
#endif
static const char *const orientationAxes[6] = {"X+", "X-", "Y+", "Y-", "Z+", "Z-"};

#define GYRO_ORIENT_LABEL_LEN 24

static void composeGyroOrientationNames(char labels[][GYRO_ORIENT_LABEL_LEN], const char *hwNames)
{
    char descriptions[6][GYRO_ORIENT_LABEL_LEN - 6] {}; // leave room for "(X+)" suffix
    if (hwNames != nullptr)
    {
        const char *start = hwNames;
        for (uint8_t i = 0; i < 6 && start != nullptr; i++)
        {
            const char *end = strchr(start, ';');
            size_t len = end ? (size_t)(end - start) : strlen(start);
            if (len > sizeof(descriptions[i]) - 1)
                len = sizeof(descriptions[i]) - 1;
            memcpy(descriptions[i], start, len);
            start = end ? end + 1 : nullptr;
        }
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        if (descriptions[i][0] != '\0')
        {
            strcpy(labels[i], descriptions[i]);
            strcat(labels[i], "(");
            strcat(labels[i], orientationAxes[i]);
            strcat(labels[i], ")");
        }
        else
            strcpy(labels[i], orientationAxes[i]);
    }
    strcpy(labels[6], "WRONG");
    strcpy(labels[7], "WRONG");
}

static struct
{
    char labels[8][GYRO_ORIENT_LABEL_LEN];
    const char *labelPtrs[8];
    char options[8 * GYRO_ORIENT_LABEL_LEN];
    bool composed;
} gyroOrientations;

/**
 * Build the labels/options once the hardware definition is available.
 * Callers may run before hardware_init() has parsed the JSON (e.g. static
 * initialisers), in which case generic axis-only labels are produced and
 * the composition is retried on the next call.
 */
static void ensureGyroOrientations()
{
    if (gyroOrientations.composed)
        return;

    const char *hwNames = GYRO_ORIENTATION_NAMES;
    composeGyroOrientationNames(gyroOrientations.labels, hwNames);
    for (uint8_t i = 0; i < 8; i++)
    {
        gyroOrientations.labelPtrs[i] = gyroOrientations.labels[i];
    }

    gyroOrientations.options[0] = '\0';
    for (uint8_t i = 0; i < 6; i++)
    {
        strcat(gyroOrientations.options, gyroOrientations.labels[i]);
        strcat(gyroOrientations.options, ";");
    }
    strcat(gyroOrientations.options, "WRONG;WRONG");

    gyroOrientations.composed = true;
}

const char *const *getGyroOrientationNames()
{
    ensureGyroOrientations();
    return gyroOrientations.labelPtrs;
}

const char *getGyroOrientationOptions()
{
    ensureGyroOrientations();
    return gyroOrientations.options;
}

/**
 * Convert a channel µs value to a float command
 *
 * This takes into account subtrim and max gyro throws.
 */
float us_command_to_float(uint8_t ch, uint16_t us)
{
    const rx_config_pwm_limits_t *limits = gyroConfig->GetPwmChannelLimits(ch);
    const uint16_t mid = limits->val.mid;
    return us <= mid
               ? float(us - mid) / (mid - limits->val.min)
               : float(us - mid) / (limits->val.max - mid);
}

/**
 * Convert +-1.0 float into µs for an output channel
 *
 * This takes into account subtrim and max gyro throws.
 */
uint16_t float_to_us(uint8_t ch, float value)
{
    const rx_config_pwm_limits_t *limits = gyroConfig->GetPwmChannelLimits(ch);
    const uint16_t mid = limits->val.mid;

    return value < 0
               ? mid + ((mid - limits->val.min) * value)
               : mid + ((limits->val.max - mid) * value);
}
#endif
