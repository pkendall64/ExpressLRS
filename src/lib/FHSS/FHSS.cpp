#include "FHSS.h"
#include "logging.h"
#include "options.h"
#include <string.h>

#if defined(UNIT_TEST)
#define POWER_OUTPUT_VALUES_COUNT 4
#define POWER_OUTPUT_VALUES_DUAL_COUNT 0
#endif

#if defined(RADIO_SX127X) || defined(RADIO_LR1121)

#if defined(RADIO_LR1121)
#include "LR1121Driver.h"
#else
#include "SX127xDriver.h"
#endif

const fhss_config_t domains[] = {
    {"AU915",  FREQ_HZ_TO_REG_VAL(915500000), FREQ_HZ_TO_REG_VAL(926900000), 20, 921000000, 0},
    {"FCC915", FREQ_HZ_TO_REG_VAL(903500000), FREQ_HZ_TO_REG_VAL(926900000), 40, 915000000, 0},
    {"EU868",  FREQ_HZ_TO_REG_VAL(863275000), FREQ_HZ_TO_REG_VAL(869575000), 13, 868000000, 0},
    {"IN866",  FREQ_HZ_TO_REG_VAL(865375000), FREQ_HZ_TO_REG_VAL(866950000), 4, 866000000, 0},
    {"AU433",  FREQ_HZ_TO_REG_VAL(433420000), FREQ_HZ_TO_REG_VAL(434420000), 3, 434000000, 0},
    {"EU433",  FREQ_HZ_TO_REG_VAL(433100000), FREQ_HZ_TO_REG_VAL(434450000), 3, 434000000, 0},
    {"US433",  FREQ_HZ_TO_REG_VAL(433250000), FREQ_HZ_TO_REG_VAL(438000000), 8, 434000000, 0},
    {"US433W", FREQ_HZ_TO_REG_VAL(423500000), FREQ_HZ_TO_REG_VAL(438000000), 20, 434000000, 0},
    {"BR915",  FREQ_HZ_TO_REG_VAL(902400000), FREQ_HZ_TO_REG_VAL(927600000), 42, 915000000, 1, {{9, 21}}},
};

#if defined(RADIO_LR1121)
const fhss_config_t domainsDualBand[] = {
    {
    #if defined(Regulatory_Domain_EU_CE_2400)
        "CE_LBT",
    #else
        "ISM2G4",
    #endif
    FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80, 2440000000, 0}
};
#endif

#elif defined(RADIO_SX128X)
#include "SX1280Driver.h"

const fhss_config_t domains[] = {
    {
    #if defined(Regulatory_Domain_EU_CE_2400)
        "CE_LBT",
    #elif defined(Regulatory_Domain_ISM_2400)
        "ISM2G4",
    #endif
    FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80, 2440000000, 0}
};
#endif

// Our table of FHSS frequencies. Define a regulatory domain to select the correct set for your location and radio
const fhss_config_t *FHSSconfig;
const fhss_config_t *FHSSconfigDualBand;

// Actual sequence of hops as indexes into the frequency list
uint8_t FHSSsequence[FHSS_SEQUENCE_LEN];
uint8_t FHSSsequence_DualBand[FHSS_SEQUENCE_LEN];

// Which entry in the sequence we currently are on
uint8_t volatile FHSSptr;

// Channel for sync packets and initial connection establishment
uint_fast8_t sync_channel;
uint_fast8_t sync_channel_DualBand;

// Offset from the predefined frequency determined by AFC on Team900 (register units)
int32_t FreqCorrection;
int32_t FreqCorrection_2;

// Frequency hop separation
uint32_t freq_spread;
uint32_t freq_spread_DualBand;

// Variable for Dual Band radios
bool FHSSusePrimaryFreqBand = true;
bool FHSSuseDualBand = false;

uint16_t primaryBandCount;
uint16_t secondaryBandCount;

constexpr uint8_t VERSION_DOMAIN_MAXLEN = 26 + 1;   // max. number of characters (plus '\0') the Lua script can display
                                                    // on color LCD radios w/o being overwritten by the commit info
char version_domain[VERSION_DOMAIN_MAXLEN] {};

static bool isChannelExcluded(uint8_t channel, const fhss_config_t *config)
{
    for (uint8_t i = 0; i < config->excluded_count; i++)
    {
        if (channel >= config->excluded_ranges[i].start && channel <= config->excluded_ranges[i].end)
        {
            return true;
        }
    }
    return false;
}

static uint32_t getUsableChannelCount(const fhss_config_t *config)
{
    uint32_t excluded = 0;
    for (uint8_t i = 0; i < config->excluded_count; i++)
    {
        excluded += (config->excluded_ranges[i].end - config->excluded_ranges[i].start + 1);
    }
    return config->freq_count - excluded;
}

void FHSSrandomiseFHSSsequence(const uint32_t seed)
{
    FHSSconfig = &domains[firmwareOptions.domain];
    freq_spread = (FHSSconfig->freq_stop - FHSSconfig->freq_start) * FREQ_SPREAD_SCALE / (FHSSconfig->freq_count - 1);

    // Find a sync channel that's not in an excluded range
    sync_channel = FHSSconfig->freq_count / 2;
    while (isChannelExcluded(sync_channel, FHSSconfig))
    {
        sync_channel = (sync_channel + 1) % FHSSconfig->freq_count;
    }

    const uint32_t usableChannels = getUsableChannelCount(FHSSconfig);
    primaryBandCount = (FHSS_SEQUENCE_LEN / usableChannels) * usableChannels;

    DBGLN("Primary Domain %s, %u/%u channels, sync=%u",
        FHSSconfig->domain, usableChannels, FHSSconfig->freq_count, sync_channel);

    FHSSrandomiseFHSSsequenceBuild(seed, FHSSconfig->freq_count, sync_channel, FHSSsequence);

#if defined(RADIO_LR1121)
    FHSSconfigDualBand = &domainsDualBand[0];
    freq_spread_DualBand = (FHSSconfigDualBand->freq_stop - FHSSconfigDualBand->freq_start) * FREQ_SPREAD_SCALE / (FHSSconfigDualBand->freq_count - 1);

    // Find a sync channel that's not in an excluded range
    sync_channel_DualBand = FHSSconfigDualBand->freq_count / 2;
    while (isChannelExcluded(sync_channel_DualBand, FHSSconfigDualBand))
    {
        sync_channel_DualBand = (sync_channel_DualBand + 1) % FHSSconfigDualBand->freq_count;
    }

    const uint32_t usableChannelsDual = getUsableChannelCount(FHSSconfigDualBand);
    secondaryBandCount = (FHSS_SEQUENCE_LEN / usableChannelsDual) * usableChannelsDual;

    DBGLN("Dual Domain %s, %u/%u channels, sync=%u",
        FHSSconfigDualBand->domain, usableChannelsDual, FHSSconfigDualBand->freq_count, sync_channel_DualBand);

    FHSSusePrimaryFreqBand = false;
    FHSSrandomiseFHSSsequenceBuild(seed, FHSSconfigDualBand->freq_count, sync_channel_DualBand, FHSSsequence_DualBand);
    FHSSusePrimaryFreqBand = true;
#endif

    // add frequency and regulatory domain to the string used by the Lua script
    addDomainInfo(version_domain, VERSION_DOMAIN_MAXLEN);
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.

*/
void FHSSrandomiseFHSSsequenceBuild(const uint32_t seed, uint32_t freqCount, uint_fast8_t syncChannel, uint8_t *inSequence)
{
    // reset the pointer (otherwise the tests fail)
    FHSSptr = 0;
    rngSeed(seed);

    const fhss_config_t *config = FHSSusePrimaryFreqBand ? FHSSconfig : FHSSconfigDualBand;

    // Build list of valid (non-excluded) channels
    uint8_t validChannels[256];
    uint8_t validCount = 0;
    for (uint8_t ch = 0; ch < freqCount; ch++)
    {
        if (!isChannelExcluded(ch, config))
        {
            validChannels[validCount++] = ch;
        }
    }

    // initialize the sequence array with valid channels only
    for (uint16_t i = 0; i < FHSSgetSequenceCount(); i++)
    {
        if (i % validCount == 0) {
            inSequence[i] = syncChannel;
        } else {
            // Use valid channels in rotation
            uint8_t validIdx = i % validCount;
            if (validChannels[validIdx] == syncChannel) {
                // Swap with first non-sync channel
                inSequence[i] = validChannels[0] == syncChannel ? validChannels[1] : validChannels[0];
            } else {
                inSequence[i] = validChannels[validIdx];
            }
        }
    }

    // Randomize the sequence (excluding sync positions)
    for (uint16_t i = 0; i < FHSSgetSequenceCount(); i++)
    {
        // if it's not the sync channel position
        if (i % validCount != 0)
        {
            uint8_t offset = (i / validCount) * validCount;   // offset to start of current block
            uint8_t rand = rngN(validCount - 1) + 1;          // random number between 1 and validCount

            // switch this entry and another random entry in the same block
            uint8_t temp = inSequence[i];
            inSequence[i] = inSequence[offset+rand];
            inSequence[offset+rand] = temp;
        }
    }

    // output FHSS sequence
    // for (uint16_t i=0; i < FHSSgetSequenceCount(); i++)
    // {
    //     DBG("%u ",inSequence[i]);
    //     if (i % 10 == 9)
    //         DBGCR;
    // }
    // DBGCR;
}

/**
 * @brief Add frequency and regulatory domain to the version string used by the Lua script. Outputs the version_domain string as:
 * [version:0..20] [subGHz domain | 2.4GHz domain] truncated to maxlen-1 for single band devices
 * [version:0..20] [subGHz domain]/[2.4GHz domain] truncated to maxlen-1 for dual band devices
 * Examples:
 *   4.0.0 CE_LBT
 *   4.1.7 AU915
 *   4.11.17 FCC915/ISM2G4
 *   someBranch EU868/CE_LBT
 *
 * @param version_domain a pointer to a buffer holding the version and extra space for additional data
 * @param maxlen the size of the provided buffer
 */
void addDomainInfo(char *version_domain, uint8_t maxlen)
{
    if (strlen(version) < 21)
    {
        strlcpy(version_domain, version, 21);
        strlcat(version_domain, " ", maxlen);
    }
    else
    {
        strlcpy(version_domain, version, 18);
        strlcat(version_domain, "... ", maxlen);
    }

    if (POWER_OUTPUT_VALUES_COUNT != 0)
    {
        strlcat(version_domain, FHSSconfig->domain, maxlen);            // single band: subghz or 2.4GHz, dual band: subghz
    }
    if (POWER_OUTPUT_VALUES_COUNT != 0 && POWER_OUTPUT_VALUES_DUAL_COUNT != 0)
    {
        strlcat(version_domain, "/", maxlen);
    }
    if (POWER_OUTPUT_VALUES_DUAL_COUNT != 0)
    {
        strlcat(version_domain, FHSSconfigDualBand->domain, maxlen);    // 2.4GHz
    }
}

bool isUsingPrimaryFreqBand()
{
    return FHSSusePrimaryFreqBand;
}