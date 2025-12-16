#if defined(TARGET_RX)

#include "config.h"
#include "rx_config_legacy.h"
#include "common.h"
#include "device.h"
#include "POWERMGNT.h"
#include "OTA.h"
#include "helpers.h"
#include "logging.h"
#include "EEPROM.h"

#if defined(PLATFORM_ESP8266)
#include "flash_hal.h"
#endif

#define CONFCOPY(member) m_config.member = old.member

RxConfig::RxConfig()
{
}

void RxConfig::Load()
{
    EEPROM.begin(1024);
    m_modified = 0;
    EEPROM.get(0, m_config);

    uint32_t version = 0;
    if ((m_config.version & CONFIG_MAGIC_MASK) == RX_CONFIG_MAGIC)
        version = m_config.version & ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    // If version is current, all done
    if (version == RX_CONFIG_VERSION)
    {
        CheckUpdateFlashedUid(false);
        return;
    }

    // Can't upgrade from version <4, or when flashing a previous version, just use defaults.
    if (version < 4 || version > RX_CONFIG_VERSION)
    {
        SetDefaults(true);
        CheckUpdateFlashedUid(true);
        return;
    }

    // Upgrade EEPROM, load defaults then load the old values into it
    SetDefaults(false);
    switch (version)
    {
        case 4:
            UpgradeEepromV4(); break;
        case 5:
            UpgradeEepromV5(); break;
        case 6:
            UpgradeEepromV6(); break;
        case 7: // fallthrough
        case 8:
            UpgradeEepromV7V8(version); break;
        case 9: // fallthrough
        case 10:
            UpgradeEepromV9V10(version); break;
    }
    m_modified = EVENT_CONFIG_MODEL_CHANGED; // anything to force write
    Commit();
}

void RxConfig::CheckUpdateFlashedUid(bool skipDescrimCheck)
{
    // No binding phrase flashed, nothing to do
    if (!firmwareOptions.hasUID)
        return;
    // If already copied binding info, do not replace
    if (!skipDescrimCheck && m_config.flash_discriminator == firmwareOptions.flash_discriminator)
        return;

    // Save the new UID along with this discriminator to prevent resetting every boot
    SetUID(firmwareOptions.uid);
    m_config.flash_discriminator = firmwareOptions.flash_discriminator;
    // Reset the power on counter because this is following a flash, may have taken a few boots to flash
    m_config.powerOnCounter = 0;
    // SetUID should set this but just in case that gets removed, flash_discriminator needs to be saved
    m_modified = EVENT_CONFIG_UID_CHANGED;

    Commit();
}

static unsigned toFailsafeV10(unsigned oldFailsafe)
{
    // the old failsafe was 988+value, new is 476+value
    return oldFailsafe + (988 - CHANNEL_VALUE_FS_US_MIN);
}

/**
 * @brief Convert rx_config_pwm_t.mode to what should be its current value, taking
 * into account every time some jerk inserted a value in the middle instead of the end
 * (eServoOutputMode)
 */
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

// ========================================================
// V4 Upgrade

static void PwmConfigV4(v4_rx_config_pwm_t const * const v4, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(v4->val.failsafe);
    current->val.inputChannel = v4->val.inputChannel;
    current->val.inverted = v4->val.inverted;
}

void RxConfig::UpgradeEepromV4()
{
    v4_rx_config_t old;
    EEPROM.get(0, old);

    UpgradeUid(nullptr, old.isBound ? old.uid : nullptr);
    CONFCOPY(modelId);
    // OG PWMP had only 8 channels
    for (unsigned ch=0; ch<8; ++ch)
        PwmConfigV4(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

// ========================================================
// V5 Upgrade

static void PwmConfigV5(v5_rx_config_pwm_t const * const v5, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(v5->val.failsafe);
    current->val.inputChannel = v5->val.inputChannel;
    current->val.inverted = v5->val.inverted;
    current->val.narrow = v5->val.narrow;
    current->val.mode = v5->val.mode;
    if (v5->val.mode > som400Hz)
    {
        current->val.mode += 1;
    }
}

void RxConfig::UpgradeEepromV5()
{
    v5_rx_config_t old;
    EEPROM.get(0, old);

    UpgradeUid(old.onLoan ? old.loanUID : nullptr, old.isBound ? old.uid : nullptr);
    m_config.vbat.scale = old.vbatScale;
    CONFCOPY(power);
    CONFCOPY(antennaMode);
    CONFCOPY(forceTlmOff);
    CONFCOPY(rateInitialIdx);
    CONFCOPY(modelId);
    for (unsigned ch=0; ch<16; ++ch)
        PwmConfigV5(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

// ========================================================
// V6 Upgrade

static void PwmConfigV6(v6_rx_config_pwm_t const * const v6, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(v6->val.failsafe);
    current->val.inputChannel = v6->val.inputChannel;
    current->val.inverted = v6->val.inverted;
    current->val.narrow = v6->val.narrow;
    current->val.mode = v6->val.mode;
}

void RxConfig::UpgradeEepromV6()
{
    v6_rx_config_t old;
    EEPROM.get(0, old);

    UpgradeUid(old.onLoan ? old.loanUID : nullptr, old.isBound ? old.uid : nullptr);
    m_config.vbat.scale = old.vbatScale;
    CONFCOPY(power);
    CONFCOPY(antennaMode);
    CONFCOPY(forceTlmOff);
    CONFCOPY(rateInitialIdx);
    CONFCOPY(modelId);
    for (unsigned ch=0; ch<16; ++ch)
        PwmConfigV6(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

// ========================================================
// V7/V8 Upgrade

void RxConfig::UpgradeEepromV7V8(uint8_t ver)
{
    v7_rx_config_t old;
    EEPROM.get(0, old);

    UpgradeUid(old.onLoan ? old.loanUID : nullptr, old.isBound ? old.uid : nullptr);
    m_config.vbat.scale = old.vbatScale;
    CONFCOPY(power);
    CONFCOPY(antennaMode);
    CONFCOPY(forceTlmOff);
    CONFCOPY(rateInitialIdx);
    CONFCOPY(modelId);
    CONFCOPY(serialProtocol);
    CONFCOPY(failsafeMode);

    for (unsigned ch=0; ch<16; ++ch)
    {
        m_config.pwmChannels[ch].raw = old.pwmChannels[ch].raw;
        m_config.pwmChannels[ch].val.mode = toServoOutputModeCurrent(ver, old.pwmChannels[ch].val.mode);
    }
}

// ========================================================
// V9 Upgrade

static void PwmConfigV9(v9_rx_config_pwm_t const * const old, rx_config_pwm_t * const current)
{
    current->val.failsafe = toFailsafeV10(old->val.failsafe);
    current->val.inputChannel = old->val.inputChannel;
    current->val.inverted = old->val.inverted;
    current->val.mode = toServoOutputModeCurrent(10, old->val.mode);
    current->val.narrow = old->val.narrow;
    current->val.failsafeMode = old->val.failsafeMode;
}

void RxConfig::UpgradeEepromV9V10(uint8_t ver)
{
    v9_rx_config_t old;
    EEPROM.get(0, old);

    UpgradeUid(nullptr, old.uid);
    // Version 10 is the main structure, version 11 changes the PWM structure
    if (ver != 10)
    {
        CONFCOPY(serial1Protocol);
        CONFCOPY(vbat.scale);
        CONFCOPY(vbat.offset);
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
    for (unsigned ch=0; ch<16; ++ch)
        PwmConfigV9(&old.pwmChannels[ch], &m_config.pwmChannels[ch]);
}

/**
 * @brief Upgrade UID and flash_discriminator from old config, using onLoanUid if != null
 */
void RxConfig::UpgradeUid(uint8_t *onLoanUid, uint8_t *boundUid)
{
    // Always set the flash_discriminator otherwise the UID might change next reboot
    m_config.flash_discriminator = firmwareOptions.flash_discriminator;
    // Convert to traditional binding
    // On loan? Now you own
    if (onLoanUid)
    {
        memcpy(m_config.uid, onLoanUid, UID_LEN);
    }
    // Compiled in UID? Bind to that
    else if (firmwareOptions.hasUID)
    {
        memcpy(m_config.uid, firmwareOptions.uid, UID_LEN);
    }
    else if (boundUid)
    {
        // keep binding
        memcpy(m_config.uid, boundUid, UID_LEN);
    }
    else
    {
        // No bind
        memset(m_config.uid, 0, UID_LEN);
    }
}

bool RxConfig::GetIsBound() const
{
    if (m_config.bindStorage == BINDSTORAGE_VOLATILE)
        return false;
    return UID_IS_BOUND(m_config.uid);
}

bool RxConfig::IsOnLoan() const
{
    if (m_config.bindStorage != BINDSTORAGE_RETURNABLE)
        return false;
    if (!firmwareOptions.hasUID)
        return false;
    return GetIsBound() && memcmp(m_config.uid, firmwareOptions.uid, UID_LEN) != 0;
}

#if defined(PLATFORM_ESP8266)
#define EMPTY_SECTOR ((FS_start - 0x1000 - 0x40200000) / SPI_FLASH_SEC_SIZE) // empty sector before FS area start
static bool erase_power_on_count = false;
static int realPowerOnCounter = -1;
uint8_t
RxConfig::GetPowerOnCounter() const
{
    if (realPowerOnCounter == -1) {
        byte zeros[16];
        ESP.flashRead(EMPTY_SECTOR * SPI_FLASH_SEC_SIZE, zeros, sizeof(zeros));
        realPowerOnCounter = sizeof(zeros);
        for (int i=0 ; i<sizeof(zeros) ; i++) {
            if (zeros[i] != 0) {
                realPowerOnCounter = i;
                break;
            }
        }
    }
    return realPowerOnCounter;
}
#endif

uint32_t
RxConfig::Commit()
{
#if defined(PLATFORM_ESP8266)
    if (erase_power_on_count)
    {
        ESP.flashEraseSector(EMPTY_SECTOR);
        erase_power_on_count = false;
    }
#endif
    if (!m_modified)
    {
        // No changes
        return 0;
    }

    // Write the struct to eeprom
    EEPROM.put(0, m_config);
    EEPROM.commit();

    uint32_t changes = m_modified;
    m_modified = 0;
    return changes;
}

// Setters
void
RxConfig::SetUID(uint8_t* uid)
{
    for (uint8_t i = 0; i < UID_LEN; ++i)
    {
        m_config.uid[i] = uid[i];
    }
    m_modified = EVENT_CONFIG_UID_CHANGED;
}

void
RxConfig::SetRelayEnabled(bool relayEnabled) {
    if (relayEnabled != m_config.relayEnabled) {
        m_config.relayEnabled = relayEnabled;
        m_modified = EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
RxConfig::SetPowerOnCounter(uint8_t powerOnCounter)
{
#if defined(PLATFORM_ESP8266)
    realPowerOnCounter = powerOnCounter;
    if (powerOnCounter == 0)
    {
        erase_power_on_count = true;
        m_modified = true;
    }
    else
    {
        byte zeros[16] = {0};
        ESP.flashWrite(EMPTY_SECTOR * SPI_FLASH_SEC_SIZE, zeros, std::min((size_t)powerOnCounter, sizeof(zeros)));
    }
#else
    if (m_config.powerOnCounter != powerOnCounter)
    {
        m_config.powerOnCounter = powerOnCounter;
        m_modified = EVENT_CONFIG_POWER_COUNT_CHANGED;
    }
#endif
}

void
RxConfig::SetModelId(uint8_t modelId)
{
    if (m_config.modelId != modelId)
    {
        m_config.modelId = modelId;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
RxConfig::SetPower(uint8_t power)
{
    if (m_config.power != power)
    {
        m_config.power = power;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}


void
RxConfig::SetAntennaMode(uint8_t antennaMode)
{
    //0 and 1 is use for gpio_antenna_select
    // 2 is diversity
    if (m_config.antennaMode != antennaMode)
    {
        m_config.antennaMode = antennaMode;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
RxConfig::SetDefaults(bool commit)
{
    // Reset everything to 0/false and then just set anything that zero is not appropriate
    memset(&m_config, 0, sizeof(m_config));

    m_config.version = RX_CONFIG_VERSION | RX_CONFIG_MAGIC;
    m_config.modelId = 0xff;
    m_config.power = POWERMGNT::getDefaultPower();
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
        m_config.antennaMode = 2; // 2 is diversity
    if (GPIO_PIN_NSS_2 != UNDEF_PIN)
        m_config.antennaMode = 0; // 0 is diversity for dual radio

    for (int ch=0; ch<PWM_MAX_CHANNELS; ++ch)
    {
        uint8_t mode = som50Hz;
        // setup defaults for hardware-defined I2C & Serial pins that are also IO pins
        if (ch < GPIO_PIN_PWM_OUTPUTS_COUNT)
        {
            if (GPIO_PIN_PWM_OUTPUTS[ch] == GPIO_PIN_SCL)
            {
                mode = somSCL;
            }
            else if (GPIO_PIN_PWM_OUTPUTS[ch] == GPIO_PIN_SDA)
            {
                mode = somSDA;
            }
            else if ((GPIO_PIN_RCSIGNAL_RX == U0RXD_GPIO_NUM && GPIO_PIN_PWM_OUTPUTS[ch] == U0RXD_GPIO_NUM) ||
                     (GPIO_PIN_RCSIGNAL_TX == U0TXD_GPIO_NUM && GPIO_PIN_PWM_OUTPUTS[ch] == U0TXD_GPIO_NUM))
            {
                mode = somSerial;
            }
#if defined(PLATFORM_ESP32)
            else if (GPIO_PIN_PWM_OUTPUTS[ch] == GPIO_PIN_SERIAL1_RX)
            {
                mode = somSerial1RX;
            }
            else if (GPIO_PIN_PWM_OUTPUTS[ch] == GPIO_PIN_SERIAL1_TX)
            {
                mode = somSerial1TX;
            }
#endif
        }
        const uint16_t failsafe = ch == 2 ? CHANNEL_VALUE_FS_US_ELIMITS_MIN - CHANNEL_VALUE_FS_US_MIN :
                                            CHANNEL_VALUE_FS_US_MID - CHANNEL_VALUE_FS_US_MIN; // ch2 is throttle, failsafe it to 880
        SetPwmChannel(ch, failsafe, ch, false, mode, false);
    }

    m_config.teamraceChannel = AUX7; // CH11

    if (commit)
    {
        // Prevent rebinding to the flashed UID on first boot
        m_config.flash_discriminator = firmwareOptions.flash_discriminator;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
        Commit();
    }
}

void
RxConfig::SetPwmChannel(uint8_t ch, uint16_t failsafe, uint8_t inputCh, bool inverted, uint8_t mode, uint8_t stretched)
{
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_pwm_t *pwm = &m_config.pwmChannels[ch];
    rx_config_pwm_t newConfig{};
    newConfig.val.failsafe = failsafe;
    newConfig.val.inputChannel = inputCh;
    newConfig.val.inverted = inverted;
    newConfig.val.mode = mode;
    newConfig.val.stretched = stretched;
    if (pwm->raw == newConfig.raw)
        return;

    pwm->raw = newConfig.raw;
    m_modified = EVENT_CONFIG_PWM_CHANGE;
}

void
RxConfig::SetPwmChannelRaw(uint8_t ch, uint32_t raw)
{
    if (ch > PWM_MAX_CHANNELS)
        return;

    rx_config_pwm_t *pwm = &m_config.pwmChannels[ch];
    if (pwm->raw == raw)
        return;

    pwm->raw = raw;
    m_modified = EVENT_CONFIG_PWM_CHANGE;
}

void
RxConfig::SetForceTlmOff(bool forceTlmOff)
{
    if (m_config.forceTlmOff != forceTlmOff)
    {
        m_config.forceTlmOff = forceTlmOff;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
RxConfig::SetRateInitialIdx(uint8_t rateInitialIdx)
{
    if (m_config.rateInitialIdx != rateInitialIdx)
    {
        m_config.rateInitialIdx = rateInitialIdx;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void RxConfig::SetSerialProtocol(eSerialProtocol serialProtocol)
{
    if (m_config.serialProtocol != serialProtocol)
    {
        m_config.serialProtocol = serialProtocol;
        m_modified = EVENT_CONFIG_SERIAL_CHANGE;
    }
}

#if defined(PLATFORM_ESP32)
void RxConfig::SetSerial1Protocol(eSerial1Protocol serialProtocol)
{
    if (m_config.serial1Protocol != serialProtocol)
    {
        m_config.serial1Protocol = serialProtocol;
        m_modified = EVENT_CONFIG_SERIAL_CHANGE;
    }
}
#endif

void RxConfig::SetTeamraceChannel(uint8_t teamraceChannel)
{
    if (m_config.teamraceChannel != teamraceChannel)
    {
        m_config.teamraceChannel = teamraceChannel;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void RxConfig::SetTeamracePosition(uint8_t teamracePosition)
{
    if (m_config.teamracePosition != teamracePosition)
    {
        m_config.teamracePosition = teamracePosition;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void RxConfig::SetFailsafeMode(eFailsafeMode failsafeMode)
{
    if (m_config.failsafeMode != failsafeMode)
    {
        m_config.failsafeMode = failsafeMode;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void RxConfig::SetBindStorage(rx_config_bindstorage_t value)
{
    if (m_config.bindStorage != value)
    {
        // If switching away from returnable, revert
        ReturnLoan();
        m_config.bindStorage = value;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void RxConfig::SetTargetSysId(uint8_t value)
{
    if (m_config.targetSysId != value)
    {
        m_config.targetSysId = value;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}
void RxConfig::SetSourceSysId(uint8_t value)
{
    if (m_config.sourceSysId != value)
    {
        m_config.sourceSysId = value;
        m_modified = EVENT_CONFIG_MODEL_CHANGED;
    }
}

void RxConfig::ReturnLoan()
{
    if (IsOnLoan())
    {
        // go back to flashed UID if there is one
        // or unbind if there is not
        if (firmwareOptions.hasUID)
            memcpy(m_config.uid, firmwareOptions.uid, UID_LEN);
        else
            memset(m_config.uid, 0, UID_LEN);

        m_modified = EVENT_CONFIG_UID_CHANGED;
    }
}

#endif