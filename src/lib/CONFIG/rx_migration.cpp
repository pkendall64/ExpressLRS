#include "rx_config_internal.h"

#include "config_legacy.h"
#include "logging.h"

#if defined(TARGET_RX)

#define CONFCOPY(member) m_config.member = old.member

static constexpr size_t LEGACY_STORAGE_SIZE = 1024U;

#if defined(PLATFORM_ESP8266)
extern "C" uint32_t _EEPROM_start;

static bool ReadLegacyRxStorageBytes(uint32_t offset, void *dst, size_t len)
{
    uint8_t *out = static_cast<uint8_t *>(dst);
    uint32_t baseAddress = reinterpret_cast<uint32_t>(&_EEPROM_start) - 0x40200000UL;
    uint32_t alignedAddress = (baseAddress + offset) & ~uint32_t(0x3U);
    size_t skip = (baseAddress + offset) - alignedAddress;
    size_t remaining = len;
    uint32_t words[8];

    while (remaining > 0)
    {
        size_t chunk = remaining + skip;
        size_t alignedLen = (chunk + 3U) & ~size_t(0x3U);
        if (alignedLen > sizeof(words))
            alignedLen = sizeof(words);
        if (!ESP.flashRead(alignedAddress, words, alignedLen))
            return false;
        size_t copyLen = alignedLen - skip;
        if (copyLen > remaining)
            copyLen = remaining;
        memcpy(out, reinterpret_cast<uint8_t *>(words) + skip, copyLen);
        out += copyLen;
        remaining -= copyLen;
        alignedAddress += static_cast<uint32_t>(alignedLen);
        skip = 0;
    }
    return true;
}
#elif defined(PLATFORM_ESP32)
static bool ReadLegacyRxStorageBytes(uint32_t offset, void *dst, size_t len)
{
    nvs_handle legacyHandle = 0;
    if (nvs_open("eeprom", NVS_READONLY, &legacyHandle) != ESP_OK)
        return false;

    uint8_t data[LEGACY_STORAGE_SIZE];
    size_t dataLen = sizeof(data);
    esp_err_t err = nvs_get_blob(legacyHandle, "eeprom", data, &dataLen);
    nvs_close(legacyHandle);
    if (err != ESP_OK || offset + len > dataLen)
        return false;

    memcpy(dst, data + offset, len);
    return true;
}
#endif

static bool ReadLegacyRxObject(void *dst, size_t len)
{
    return ReadLegacyRxStorageBytes(0, dst, len);
}

static unsigned toFailsafeV10(unsigned oldFailsafe)
{
    // the old failsafe was 988+value, new is 476+value
    return oldFailsafe + (US_CHANNEL_VALUE_STD_MIN - US_CHANNEL_VALUE_MIN);
}

static uint8_t toServoOutputModeCurrent(uint8_t verStart, uint8_t mode)
{
    // somDShot
    if (verStart < 8 && mode > somOnOff)
        mode += 1;
    // somDShot3D
    if (verStart < 11 && mode > somDShot)
        mode += 1;
    return mode;
}

static void PwmConfigV4(v4_rx_config_pwm_t const * const v4, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(v4->val.failsafe);
    current->val.inputChannel = v4->val.inputChannel;
    current->val.inverted = v4->val.inverted;
}

static void PwmConfigV5(v5_rx_config_pwm_t const * const v5, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(v5->val.failsafe);
    current->val.inputChannel = v5->val.inputChannel;
    current->val.inverted = v5->val.inverted;
    current->val.narrow = v5->val.narrow;
    current->val.mode = v5->val.mode;
    if (v5->val.mode > som400Hz)
        current->val.mode += 1;
}

static void PwmConfigV6(v6_rx_config_pwm_t const * const v6, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(v6->val.failsafe);
    current->val.inputChannel = v6->val.inputChannel;
    current->val.inverted = v6->val.inverted;
    current->val.narrow = v6->val.narrow;
    current->val.mode = v6->val.mode;
}

static void PwmConfigV9(v9_rx_config_pwm_t const * const old, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(old->val.failsafe);
    current->val.inputChannel = old->val.inputChannel;
    current->val.inverted = old->val.inverted;
    current->val.mode = toServoOutputModeCurrent(10, old->val.mode);
    current->val.narrow = old->val.narrow;
    current->val.failsafeMode = old->val.failsafeMode;
}

bool RxConfig::MigrateLegacyConfig()
{
#if defined(PLATFORM_ESP8266)
    flash_nvs_set_esp8266_default_config();
    if (flash_nvs_has_store())
        return false;
#endif

    uint32_t legacyVersion = 0;
    if (ReadLegacyRxStorageBytes(0, &legacyVersion, sizeof(legacyVersion)) && ((legacyVersion & CONFIG_MAGIC_MASK) == RX_CONFIG_MAGIC))
        legacyVersion &= ~CONFIG_MAGIC_MASK;
    else
        legacyVersion = 0;

    DBGLN("Legacy config version %u", legacyVersion);

    if (legacyVersion < 4 || legacyVersion > 11)
        return false;

    if (legacyVersion == 11)
    {
        v11_rx_config_t old;
        if (!ReadLegacyRxStorageBytes(0, &old, sizeof(old)))
            return false;
        memcpy(m_config.uid, old.uid, UID_LEN);
        m_config.serial1Protocol = old.serial1Protocol;
        m_config.flash_discriminator = old.flash_discriminator;
        m_config.bindStorage = old.bindStorage;
        m_config.power = old.power;
        m_config.antennaMode = old.antennaMode;
        m_config.powerOnCounter = old.powerOnCounter;
        m_config.forceTlmOff = old.forceTlmOff;
        m_config.rateInitialIdx = old.rateInitialIdx;
        m_config.modelId = old.modelId;
        m_config.serialProtocol = old.serialProtocol;
        m_config.failsafeMode = old.failsafeMode;
        m_config.antennaGroup = old.antennaGroup;
        for (unsigned ch = 0; ch < PWM_MAX_CHANNELS; ++ch)
            m_config.pwmChannels[ch].raw = old.pwmChannels[ch].raw;
        m_config.teamraceChannel = old.teamraceChannel;
        m_config.teamracePosition = old.teamracePosition;
        m_config.teamracePitMode = old.teamracePitMode;
        m_config.targetSysId = old.targetSysId;
        m_config.sourceSysId = old.sourceSysId;
    }
    else
    {
        SetDefaults(false);
        switch (legacyVersion)
        {
            case 4:
                UpgradeEepromV4(); break;
            case 5:
                UpgradeEepromV5(); break;
            case 6:
                UpgradeEepromV6(); break;
            case 7:
            case 8:
                UpgradeEepromV7V8(legacyVersion); break;
            case 9:
            case 10:
                UpgradeEepromV9V10(legacyVersion); break;
        }
    }

    if (firmwareOptions.hasUID && m_config.flash_discriminator != firmwareOptions.flash_discriminator)
    {
        memcpy(m_config.uid, firmwareOptions.uid, UID_LEN);
        m_config.flash_discriminator = firmwareOptions.flash_discriminator;
    }

    m_config.powerOnCounter = 0;
    m_config.version = RX_CONFIG_VERSION | RX_CONFIG_MAGIC;
    m_modified = ALL_CHANGED;
    m_pwmDirtyMask = 0xFFFFU;
    return true;
}

void RxConfig::UpgradeEepromV4()
{
    v4_rx_config_t old;
    ReadLegacyRxObject(&old, sizeof(old));

    UpgradeUid(nullptr, old.isBound ? old.uid : nullptr);
    CONFCOPY(modelId);
    // OG PWMP had only 8 channels
    for (unsigned ch=0; ch<8; ++ch)
        PwmConfigV4(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

void RxConfig::UpgradeEepromV5()
{
    v5_rx_config_t old;
    ReadLegacyRxObject(&old, sizeof(old));

    UpgradeUid(old.onLoan ? old.loanUID : nullptr, old.isBound ? old.uid : nullptr);
    CONFCOPY(power);
    CONFCOPY(antennaMode);
    CONFCOPY(forceTlmOff);
    CONFCOPY(rateInitialIdx);
    CONFCOPY(modelId);
    for (unsigned ch=0; ch<16; ++ch)
        PwmConfigV5(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

void RxConfig::UpgradeEepromV6()
{
    v6_rx_config_t old;
    ReadLegacyRxObject(&old, sizeof(old));

    UpgradeUid(old.onLoan ? old.loanUID : nullptr, old.isBound ? old.uid : nullptr);
    CONFCOPY(power);
    CONFCOPY(antennaMode);
    CONFCOPY(forceTlmOff);
    CONFCOPY(rateInitialIdx);
    CONFCOPY(modelId);
    for (unsigned ch=0; ch<16; ++ch)
        PwmConfigV6(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

void RxConfig::UpgradeEepromV7V8(uint8_t ver)
{
    v7_rx_config_t old;
    ReadLegacyRxObject(&old, sizeof(old));

    UpgradeUid(old.onLoan ? old.loanUID : nullptr, old.isBound ? old.uid : nullptr);
    CONFCOPY(power);
    CONFCOPY(antennaMode);
    CONFCOPY(forceTlmOff);
    CONFCOPY(rateInitialIdx);
    CONFCOPY(modelId);
    CONFCOPY(serialProtocol);
    CONFCOPY(failsafeMode);

    for (unsigned ch = 0; ch < 16; ++ch)
    {
        m_config.pwmChannels[ch].raw = old.pwmChannels[ch].raw;
        m_config.pwmChannels[ch].val.failsafe = toFailsafeV10(old.pwmChannels[ch].val.failsafe);
        m_config.pwmChannels[ch].val.inputChannel = old.pwmChannels[ch].val.inputChannel;
        m_config.pwmChannels[ch].val.inverted = old.pwmChannels[ch].val.inverted;
        m_config.pwmChannels[ch].val.mode = toServoOutputModeCurrent(ver, old.pwmChannels[ch].val.mode);
        m_config.pwmChannels[ch].val.narrow = old.pwmChannels[ch].val.narrow;
    }
}

void RxConfig::UpgradeEepromV9V10(uint8_t ver)
{
    v9_rx_config_t old;
    ReadLegacyRxObject(&old, sizeof(old));

    UpgradeUid(nullptr, old.uid);
    // Version 10 is the main structure, version 11 changes the PWM structure
    if (ver != 10)
    {
        CONFCOPY(serial1Protocol);
        CONFCOPY(bindStorage);
        CONFCOPY(power);
        CONFCOPY(antennaMode);
        CONFCOPY(forceTlmOff);
        CONFCOPY(rateInitialIdx);
        CONFCOPY(modelId);
        CONFCOPY(serialProtocol);
        CONFCOPY(failsafeMode);
        CONFCOPY(teamraceChannel);
        CONFCOPY(teamracePosition);
        CONFCOPY(teamracePitMode);
        CONFCOPY(targetSysId);
        CONFCOPY(sourceSysId);
    }
    for (unsigned ch = 0; ch < 16; ++ch)
        PwmConfigV9(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

#endif
