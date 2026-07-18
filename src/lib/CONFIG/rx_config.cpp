#include "rx_config_internal.h"

#include "POWERMGNT.h"
#include "logging.h"

#if defined(PLATFORM_ESP8266)
#include "flash_hal.h"
#endif

#if defined(TARGET_RX)

RxConfig::RxConfig()
    : BindphraseConfigurable()
    , m_modified(0)
    , m_pwmDirtyMask(0)
{
}

void RxConfig::Load()
{
    m_modified = 0;
    m_pwmDirtyMask = 0;

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        ERRLN("RxConfig NVS init failed");
        SetDefaults(false);
        return;
    }
    if (nvs_open("ELRS", NVS_READWRITE, &handle) != ESP_OK)
    {
        ERRLN("RxConfig NVS open failed");
        SetDefaults(false);
        return;
    }

    uint32_t version = 0;
    if (nvs_get_u32(handle, RX_NVS_KEY_VERSION, &version) == ESP_OK && ((version & CONFIG_MAGIC_MASK) == RX_CONFIG_MAGIC))
        version &= ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    if (version == RX_CONFIG_VERSION)
    {
        SetDefaults(false);

        rx_identity_storage_t identity{};
        size_t identityLen = sizeof(identity);
        if (nvs_get_blob(handle, RX_NVS_KEY_IDENTITY, &identity, &identityLen) == ESP_OK && identityLen == sizeof(identity))
        {
            memcpy(m_config.uid, identity.uid, UID_LEN);
            m_config.flash_discriminator = identity.flash_discriminator;
            m_config.bindStorage = identity.bindStorage;
        }

        uint8_t powerOnCounter = 0;
        if (nvs_get_u8(handle, RX_NVS_KEY_POWER_ON_COUNT, &powerOnCounter) == ESP_OK)
            m_config.powerOnCounter = powerOnCounter;

        rx_main_storage_t main{};
        size_t mainLen = sizeof(main);
        if (nvs_get_blob(handle, RX_NVS_KEY_MAIN, &main, &mainLen) == ESP_OK && mainLen == sizeof(main))
        {
            m_config.power = main.power;
            m_config.antennaMode = main.antennaMode;
            m_config.antennaGroup = main.antennaGroup;
            m_config.forceTlmOff = main.forceTlmOff;
            m_config.rateInitialIdx = main.rateInitialIdx;
            m_config.modelId = main.modelId;
            m_config.failsafeMode = main.failsafeMode;
            m_config.teamraceChannel = main.teamraceChannel;
            m_config.teamracePosition = main.teamracePosition;
            m_config.teamracePitMode = main.teamracePitMode;
            m_config.targetSysId = main.targetSysId;
            m_config.sourceSysId = main.sourceSysId;
        }

        rx_serial_storage_t serial{};
        size_t serialLen = sizeof(serial);
        if (nvs_get_blob(handle, RX_NVS_KEY_SERIAL, &serial, &serialLen) == ESP_OK && serialLen == sizeof(serial))
        {
            m_config.serialProtocol = serial.serialProtocol;
            m_config.serial1Protocol = serial.serial1Protocol;
        }

        for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ++ch)
        {
            RX_PWM_KEY_DECLARE(pwmKey, ch);
            uint32_t raw = 0;
            if (nvs_get_u32(handle, RX_PWM_KEY(pwmKey), &raw) == ESP_OK)
                m_config.pwmChannels[ch].raw = raw;
        }

        CheckUpdateFlashedUid(false);
        return;
    }

    if (MigrateLegacyConfig())
    {
        Commit();
        return;
    }

    SetDefaults(true);
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
    // SetUID should set this but just in case that gets removed, flash_discriminator and power-on count need to be saved
    m_modified = EVENT_CONFIG_UID_CHANGED | EVENT_CONFIG_POWER_COUNT_CHANGED;

    Commit();
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
    return OtaUidIsBound(m_config.uid);
}

bool RxConfig::IsOnLoan() const
{
    if (m_config.bindStorage != BINDSTORAGE_RETURNABLE)
        return false;
    if (!firmwareOptions.hasUID)
        return false;
    return GetIsBound() && memcmp(m_config.uid, firmwareOptions.uid, UID_LEN) != 0;
}

uint32_t
RxConfig::Commit()
{
    if (!m_modified)
    {
        DBGLN("No changes");
        return 0;
    }

    if (m_modified & EVENT_CONFIG_UID_CHANGED)
    {
        rx_identity_storage_t identity{};
        memcpy(identity.uid, m_config.uid, UID_LEN);
        identity.flash_discriminator = m_config.flash_discriminator;
        identity.bindStorage = m_config.bindStorage;
        nvs_set_blob(handle, RX_NVS_KEY_IDENTITY, &identity, sizeof(identity));
    }

    if (m_modified & EVENT_CONFIG_POWER_COUNT_CHANGED)
    {
        nvs_set_u8(handle, RX_NVS_KEY_POWER_ON_COUNT, m_config.powerOnCounter);
    }

    if (m_modified & EVENT_CONFIG_MAIN_CHANGED)
    {
        rx_main_storage_t main{};
        main.power = m_config.power;
        main.antennaMode = m_config.antennaMode;
        main.antennaGroup = m_config.antennaGroup;
        main.forceTlmOff = m_config.forceTlmOff;
        main.rateInitialIdx = m_config.rateInitialIdx;
        main.modelId = m_config.modelId;
        main.failsafeMode = m_config.failsafeMode;
        main.teamraceChannel = m_config.teamraceChannel;
        main.teamracePosition = m_config.teamracePosition;
        main.teamracePitMode = m_config.teamracePitMode;
        main.targetSysId = m_config.targetSysId;
        main.sourceSysId = m_config.sourceSysId;
        nvs_set_blob(handle, RX_NVS_KEY_MAIN, &main, sizeof(main));
    }

    if (m_modified & EVENT_CONFIG_SERIAL_CHANGE)
    {
        rx_serial_storage_t serial{};
        serial.serialProtocol = m_config.serialProtocol;
        serial.serial1Protocol = m_config.serial1Protocol;
        nvs_set_blob(handle, RX_NVS_KEY_SERIAL, &serial, sizeof(serial));
    }

    if (m_modified & EVENT_CONFIG_PWM_CHANGE)
    {
        for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ++ch)
        {
            RX_PWM_KEY_DECLARE(pwmKey, ch);
            if (m_pwmDirtyMask & (1U << ch))
                nvs_set_u32(handle, RX_PWM_KEY(pwmKey), m_config.pwmChannels[ch].raw);
        }
    }

    if (m_modified & EVENT_CONFIG_VERSION_CHANGED)
    {
        nvs_set_u32(handle, RX_NVS_KEY_VERSION, m_config.version);
    }

    nvs_commit(handle);

    uint32_t changes = m_modified;
    m_modified = 0;
    m_pwmDirtyMask = 0;
    return changes;
}

// Setters
void
RxConfig::SetUID(uint8_t uid[UID_LEN])
{
    for (uint8_t i = 0; i < UID_LEN; ++i)
    {
        m_config.uid[i] = uid[i];
    }
    m_modified |= EVENT_CONFIG_UID_CHANGED;
}

void
RxConfig::SetPowerOnCounter(uint8_t powerOnCounter)
{
    if (m_config.powerOnCounter != powerOnCounter)
    {
        m_config.powerOnCounter = powerOnCounter;
        m_modified |= EVENT_CONFIG_POWER_COUNT_CHANGED;
    }
}

void
RxConfig::SetModelId(uint8_t modelId)
{
    if (m_config.modelId != modelId)
    {
        m_config.modelId = modelId;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
RxConfig::SetPower(uint8_t power)
{
    if (m_config.power != power)
    {
        m_config.power = power;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
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
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
RxConfig::SetAntennaGroup(uint8_t antennaGroup)
{
    if (m_config.antennaGroup != antennaGroup)
    {
        m_config.antennaGroup = antennaGroup;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
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
    m_pwmDirtyMask = 0;

    for (int ch=0; ch<PWM_MAX_CHANNELS; ++ch)
    {
        uint8_t mode = som50Hz;
        // setup defaults for hardware-defined I2C & Serial pins that are also IO pins
        if (!OPT_PWM_OUT_ONLY && ch < GPIO_PIN_PWM_OUTPUTS_COUNT)
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
        const uint16_t failsafe = ch == 2 ? US_CHANNEL_VALUE_EXT_MIN - US_CHANNEL_VALUE_MIN :
                                            US_CHANNEL_VALUE_CENTER - US_CHANNEL_VALUE_MIN; // ch2 is throttle, failsafe it to 880
        SetPwmChannel(ch, failsafe, ch, false, mode, false);
    }

    m_config.teamraceChannel = AUX7; // CH11

    if (commit)
    {
        // Prevent rebinding to the flashed UID on first boot
        m_config.flash_discriminator = firmwareOptions.flash_discriminator;
        m_modified = ALL_CHANGED;
        m_pwmDirtyMask = 0xFFFFU;
        Commit();
    }
    else
    {
        m_modified = 0;
        m_pwmDirtyMask = 0;
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
    m_modified |= EVENT_CONFIG_PWM_CHANGE;
    m_pwmDirtyMask |= (1U << ch);
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
    m_modified |= EVENT_CONFIG_PWM_CHANGE;
    m_pwmDirtyMask |= (1U << ch);
}

void
RxConfig::SetForceTlmOff(bool forceTlmOff)
{
    if (m_config.forceTlmOff != forceTlmOff)
    {
        m_config.forceTlmOff = forceTlmOff;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
RxConfig::SetRateInitialIdx(uint8_t rateInitialIdx)
{
    if (m_config.rateInitialIdx != rateInitialIdx)
    {
        m_config.rateInitialIdx = rateInitialIdx;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void RxConfig::SetSerialProtocol(eSerialProtocol serialProtocol)
{
    if (m_config.serialProtocol != serialProtocol)
    {
        m_config.serialProtocol = serialProtocol;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_SERIAL_CHANGE;
    }
}

#if defined(PLATFORM_ESP32)
void RxConfig::SetSerial1Protocol(eSerial1Protocol serialProtocol)
{
    if (m_config.serial1Protocol != serialProtocol)
    {
        m_config.serial1Protocol = serialProtocol;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_SERIAL_CHANGE;
    }
}
#endif

void RxConfig::SetTeamraceChannel(uint8_t teamraceChannel)
{
    if (m_config.teamraceChannel != teamraceChannel)
    {
        m_config.teamraceChannel = teamraceChannel;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void RxConfig::SetTeamracePosition(uint8_t teamracePosition)
{
    if (m_config.teamracePosition != teamracePosition)
    {
        m_config.teamracePosition = teamracePosition;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void RxConfig::SetFailsafeMode(eFailsafeMode failsafeMode)
{
    if (m_config.failsafeMode != failsafeMode)
    {
        m_config.failsafeMode = failsafeMode;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void RxConfig::SetBindStorage(rx_config_bindstorage_t value)
{
    if (m_config.bindStorage != value)
    {
        // If switching away from returnable, revert
        ReturnLoan();
        m_config.bindStorage = value;
        m_modified |= EVENT_CONFIG_UID_CHANGED;
    }
}

void RxConfig::SetTargetSysId(uint8_t value)
{
    if (m_config.targetSysId != value)
    {
        m_config.targetSysId = value;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}
void RxConfig::SetSourceSysId(uint8_t value)
{
    if (m_config.sourceSysId != value)
    {
        m_config.sourceSysId = value;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
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
        m_modified |= EVENT_CONFIG_UID_CHANGED;
    }
}

#endif
