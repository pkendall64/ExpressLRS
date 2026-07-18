#include "tx_config_internal.h"

#include "config_legacy.h"
#include "logging.h"

#if defined(TARGET_TX)

static constexpr size_t LEGACY_STORAGE_SIZE = 1024U;

#if defined(PLATFORM_ESP8266)
extern "C" uint32_t _EEPROM_start;

static bool ReadLegacyTxStorageBytes(uint32_t offset, void *dst, size_t len)
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
static bool ReadLegacyTxStorageBytes(uint32_t offset, void *dst, size_t len)
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

static uint8_t RateV6toV7(uint8_t rateV6)
{
#if defined(RADIO_SX127X) || defined(RADIO_LR1121)
    if (rateV6 == 0)
    {
        // 200Hz stays same
        return 0;
    }

    // 100Hz, 50Hz, 25Hz all move up one
    // to make room for 100Hz Full
    return rateV6 + 1;
#else // RADIO_2400
    switch (rateV6)
    {
        case 0: return 4; // 500Hz
        case 1: return 6; // 250Hz
        case 2: return 7; // 150Hz
        case 3: return 9; // 50Hz
        default: return 4; // 500Hz
    }
#endif // RADIO_2400
}

static uint8_t RatioV6toV7(uint8_t ratioV6)
{
    // All shifted up for Std telem
    return ratioV6 + 1;
}

static uint8_t SwitchesV6toV7(uint8_t switchesV6)
{
    // 0 was removed, Wide(2) became 0, Hybrid(1) became 1
    switch (switchesV6)
    {
        case 1: return (uint8_t)smHybridOr16ch;
        case 2:
        default:
            return (uint8_t)smWideOr8ch;
    }
}

static void ModelV6toV7(v6_model_config_t const * const v6, v7_model_config_t * const v7)
{
    v7->rate = RateV6toV7(v6->rate);
    v7->tlm = RatioV6toV7(v6->tlm);
    v7->power = v6->power;
    v7->switchMode = SwitchesV6toV7(v6->switchMode);
    v7->modelMatch = v6->modelMatch;
    v7->dynamicPower = v6->dynamicPower;
    v7->boostChannel = v6->boostChannel;
}

static void ModelV7toV8(v7_model_config_t const * const v7, model_config_t * const v8)
{
    uint8_t newRate = v7->rate;
#if defined(RADIO_LR1121)
    switch (newRate)
    {
        case 0: newRate = 3; break; // lora 900 200Hz
        case 1: newRate = 4; break; // lora 900 100Hz Full
        case 2: newRate = 5; break; // lora 900 100Hz
        case 3: newRate = 6; break; // lora 900 50Hz
        case 4: newRate = 12; break; // lora 2.4 500Hz
        case 5: newRate = 13; break; // lora 2.4 333Hz Full
        case 6: newRate = 14; break; // lora 2.4 250Hz
        case 7: newRate = 15; break; // lora 2.4 150Hz
        case 8: newRate = 16; break; // lora 2.4 100Hz Full
        case 9: newRate = 17; break; // lora 2.4 50Hz
        case 10: newRate = 18; break; // lora dual 150Hz
        case 11: newRate = 19; break; // lora dual 100Hz Full
        case 12: newRate = 1; break; // lora 900 250Hz
        case 13: newRate = 2; break; // lora 900 200Hz Full
        case 14: newRate = 10; break; // fsk 2.4 500Hz dvda
        case 15: newRate = 0; break; // fsk 900 1000Hz
    }
#endif
    v8->rate = newRate;
    v8->tlm = v7->tlm;
    v8->power = v7->power;
    v8->switchMode = v7->switchMode;
    v8->boostChannel = v7->boostChannel;
    v8->dynamicPower = v7->dynamicPower;
    v8->modelMatch = v7->modelMatch;
    v8->txAntenna = v7->txAntenna;
    v8->ptrStartChannel = v7->ptrStartChannel;
    v8->ptrEnableChannel = v7->ptrEnableChannel;
    v8->linkMode = v7->linkMode;
}

bool TxConfig::MigrateLegacyConfig()
{
    uint32_t legacyVersion = 0;
    if (nvs_get_u32(handle, TX_NVS_KEY_VERSION, &legacyVersion) == ESP_OK && ((legacyVersion & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC))
        legacyVersion &= ~CONFIG_MAGIC_MASK;
    else
        legacyVersion = 0;

    if (legacyVersion >= 5 && legacyVersion < TX_CONFIG_VERSION)
    {
        DBGLN("Migrating NVS config version %u", legacyVersion);
        SetDefaults(false);

        uint32_t value = 0;
        uint8_t value8 = 0;

        if (nvs_get_u32(handle, TX_NVS_KEY_VTX, &value) == ESP_OK)
        {
            m_config.vtxBand = value >> 24;
            m_config.vtxChannel = value >> 16;
            m_config.vtxPower = value >> 8;
            m_config.vtxPitmode = value;
        }
        if (nvs_get_u8(handle, TX_NVS_KEY_FANTHRESH, &value8) == ESP_OK)
            m_config.powerFanThreshold = value8;
        if (nvs_get_u32(handle, TX_NVS_KEY_FAN, &value) == ESP_OK)
            m_config.fanMode = value;
        if (nvs_get_u32(handle, TX_NVS_KEY_MOTION, &value) == ESP_OK)
            m_config.motionMode = value;
        if (legacyVersion >= 6)
        {
            if (nvs_get_u8(handle, TX_NVS_KEY_DVRAUX, &value8) == ESP_OK)
                m_config.dvrAux = value8;
            if (nvs_get_u8(handle, TX_NVS_KEY_DVRSTARTDELAY, &value8) == ESP_OK)
                m_config.dvrStartDelay = value8;
            if (nvs_get_u8(handle, TX_NVS_KEY_DVRSTOPDELAY, &value8) == ESP_OK)
                m_config.dvrStopDelay = value8;
        }
        if (legacyVersion >= 7)
        {
            if (nvs_get_u32(handle, TX_NVS_KEY_BUTTON1, &value) == ESP_OK)
                m_config.buttonColors[0].raw = value;
            if (nvs_get_u32(handle, TX_NVS_KEY_BUTTON2, &value) == ESP_OK)
                m_config.buttonColors[1].raw = value;
            if (nvs_get_u8(handle, TX_NVS_KEY_BACKPACKDISABLE, &value8) == ESP_OK)
                m_config.backpackDisable = value8;
            if (nvs_get_u8(handle, TX_NVS_KEY_BACKPACKTLMEN, &value8) == ESP_OK)
                m_config.backpackTlmMode = value8;
        }

        for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; ++i)
        {
            TX_MODEL_KEY_DECLARE(modelKey, i);
            if (nvs_get_u32(handle, TX_MODEL_KEY(modelKey), &value) != ESP_OK)
                continue;

            if (legacyVersion == 6)
            {
                v6_model_config_t v6Model;
                v7_model_config_t v7Model;
                U32_to_Model(value, &v6Model);
                ModelV6toV7(&v6Model, &v7Model);
                ModelV7toV8(&v7Model, &m_config.model_config[i]);
            }
            else
            {
                v7_model_config_t v7Model;
                U32_to_Model(value, &v7Model);
                ModelV7toV8(&v7Model, &m_config.model_config[i]);
            }

            if (!isSupportedRFRate(m_config.model_config[i].rate))
                m_config.model_config[i].rate = enumRatetoIndexSafe(POWER_OUTPUT_VALUES_COUNT == 0 ? RATE_LORA_2G4_250HZ : RATE_LORA_900_200HZ);
        }

        m_modified = ALL_CHANGED;
        m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
        return true;
    }

#if defined(PLATFORM_ESP8266)
    flash_nvs_set_esp8266_default_config();
    if (flash_nvs_has_store())
        return false;
#endif

    if (ReadLegacyTxStorageBytes(0, &legacyVersion, sizeof(legacyVersion)) && ((legacyVersion & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC))
        legacyVersion &= ~CONFIG_MAGIC_MASK;
    else
        legacyVersion = 0;

    DBGLN("Legacy config version %u", legacyVersion);

    if (legacyVersion == TX_CONFIG_VERSION)
    {
        if (!ReadLegacyTxStorageBytes(0, &m_config, sizeof(m_config)))
            return false;
        m_modified = ALL_CHANGED;
        m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
        return true;
    }
    if (legacyVersion == 5)
    {
        SetDefaults(false);
        UpgradeEepromV5ToV6();
        return true;
    }
    if (legacyVersion == 6)
    {
        SetDefaults(false);
        UpgradeEepromV6ToV7();
        return true;
    }
    if (legacyVersion == 7)
    {
        SetDefaults(false);
        UpgradeEepromV7ToV8();
        return true;
    }

    return false;
}

void TxConfig::UpgradeEepromV5ToV6()
{
    v5_tx_config_t v5Config;
    v6_tx_config_t v6Config = { 0 };
    v7_tx_config_t v7Config = { 0 };

    ReadLegacyTxStorageBytes(0, &v5Config, sizeof(v5Config));
    memcpy(&v6Config, &v5Config, sizeof(v5Config));
    v6Config.version = 6U | TX_CONFIG_MAGIC;

    #define LAZY(member) v7Config.member = v6Config.member
    LAZY(vtxBand);
    LAZY(vtxChannel);
    LAZY(vtxPower);
    LAZY(vtxPitmode);
    LAZY(powerFanThreshold);
    LAZY(fanMode);
    LAZY(motionMode);
    LAZY(dvrAux);
    LAZY(dvrStartDelay);
    LAZY(dvrStopDelay);
    #undef LAZY

    for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; i++)
        ModelV6toV7(&v6Config.model_config[i], &v7Config.model_config[i]);

    #define LAZY(member) m_config.member = v7Config.member
    LAZY(vtxBand);
    LAZY(vtxChannel);
    LAZY(vtxPower);
    LAZY(vtxPitmode);
    LAZY(powerFanThreshold);
    LAZY(fanMode);
    LAZY(motionMode);
    LAZY(dvrAux);
    LAZY(dvrStartDelay);
    LAZY(dvrStopDelay);
    #undef LAZY

    for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; i++)
        ModelV7toV8(&v7Config.model_config[i], &m_config.model_config[i]);

    m_modified = ALL_CHANGED;
    m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
}

void TxConfig::UpgradeEepromV6ToV7()
{
    v6_tx_config_t v6Config;
    v7_tx_config_t v7Config = { 0 };

    ReadLegacyTxStorageBytes(0, &v6Config, sizeof(v6Config));

    #define LAZY(member) v7Config.member = v6Config.member
    LAZY(vtxBand);
    LAZY(vtxChannel);
    LAZY(vtxPower);
    LAZY(vtxPitmode);
    LAZY(powerFanThreshold);
    LAZY(fanMode);
    LAZY(motionMode);
    LAZY(dvrAux);
    LAZY(dvrStartDelay);
    LAZY(dvrStopDelay);
    #undef LAZY

    for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; i++)
        ModelV6toV7(&v6Config.model_config[i], &v7Config.model_config[i]);

    #define LAZY(member) m_config.member = v7Config.member
    LAZY(vtxBand);
    LAZY(vtxChannel);
    LAZY(vtxPower);
    LAZY(vtxPitmode);
    LAZY(powerFanThreshold);
    LAZY(fanMode);
    LAZY(motionMode);
    LAZY(dvrAux);
    LAZY(dvrStartDelay);
    LAZY(dvrStopDelay);
    #undef LAZY

    for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; i++)
        ModelV7toV8(&v7Config.model_config[i], &m_config.model_config[i]);

    m_modified = ALL_CHANGED;
    m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
}

void TxConfig::UpgradeEepromV7ToV8()
{
    v7_tx_config_t v7Config;
    ReadLegacyTxStorageBytes(0, &v7Config, sizeof(v7Config));

    #define LAZY(member) m_config.member = v7Config.member
    LAZY(vtxBand);
    LAZY(vtxChannel);
    LAZY(vtxPower);
    LAZY(vtxPitmode);
    LAZY(powerFanThreshold);
    LAZY(fanMode);
    LAZY(motionMode);
    LAZY(dvrAux);
    LAZY(dvrStartDelay);
    LAZY(dvrStopDelay);
    #undef LAZY

    for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; i++)
        ModelV7toV8(&v7Config.model_config[i], &m_config.model_config[i]);

    m_modified = ALL_CHANGED;
    m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
}

#endif
