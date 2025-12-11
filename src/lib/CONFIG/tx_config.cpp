#if defined(TARGET_TX)

#include "config.h"
#include "tx_config_legacy.h"
#include "common.h"
#include "device.h"
#include "POWERMGNT.h"
#include "OTA.h"
#include "helpers.h"
#include "logging.h"
#include "EEPROM.h"

#define ALL_CHANGED         (EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_VTX_CHANGED | EVENT_CONFIG_MAIN_CHANGED | EVENT_CONFIG_FAN_CHANGED | EVENT_CONFIG_MOTION_CHANGED | EVENT_CONFIG_BUTTON_CHANGED | EVENT_CONFIG_VERSION_CHANGED)

// Really awful but safe(?) type punning of model_config_t/v6_model_config_t to and from uint32_t
template<class T> static void U32_to_Model(uint32_t const u32, T *const model)
{
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint32_t)-sizeof(T)];
        } val;
        uint32_t u32;
    } converter = { .u32 = u32 };

    *model = converter.val.model;
}

template<class T> static uint32_t Model_to_U32(T const *const model)
{
    // clear the entire union because the assignment will only fill sizeof(T)
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint32_t)-sizeof(T)];
        } val;
        uint32_t u32;
    } converter = { .val = {.model = *model } };

    return converter.u32;
}

template<class T> static void U64_to_Model(uint64_t const u64, T *const model)
{
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint64_t)-sizeof(T)];
        } val;
        uint64_t u64;
    } converter = { .u64 = u64 };

    *model = converter.val.model;
}

template<class T> static uint64_t Model_to_U64(T const *const model)
{
    // clear the entire union because the assignment will only fill sizeof(T)
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint64_t)-sizeof(T)];
        } val;
        uint64_t u64;
    } converter = { .val = {.model = *model } };

    return converter.u64;
}

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

static void ModelV7toV8(v7_model_config_t const * const v7, v8_model_config_t * const v8)
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

static void ModelV8toV9(v8_model_config_t const * const v8, model_config_t * const v9)
{
    v9->rate = v8->rate;
    v9->tlm = v8->tlm;
    v9->power = v8->power;
    v9->switchMode = v8->switchMode;
    v9->boostChannel = v8->boostChannel;
    v9->dynamicPower = v8->dynamicPower;
    v9->modelMatch = v8->modelMatch;
    v9->txAntenna = v8->txAntenna;
    v9->ptrStartChannel = v8->ptrStartChannel;
    v9->ptrEnableChannel = v8->ptrEnableChannel;
    v9->linkMode = v8->linkMode;
    v9->relay = false;
}

TxConfig::TxConfig() :
    m_model(m_config.model_config)
{
}

#if defined(PLATFORM_ESP32)
void TxConfig::Load()
{
    m_modified = 0;

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    ESP_ERROR_CHECK(nvs_open("ELRS", NVS_READWRITE, &handle));

    // Try to load the version and make sure it is a TX config
    uint32_t version = 0;
    if (nvs_get_u32(handle, "tx_version", &version) == ESP_OK && ((version & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC))
        version = version & ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    // Can't upgrade from version <5, or when flashing a previous version, just use defaults.
    if (version < 5 || version > TX_CONFIG_VERSION)
    {
        SetDefaults(true);
        return;
    }

    SetDefaults(false);

    uint32_t value;
    uint8_t value8;
    // vtx (v5)
    if (nvs_get_u32(handle, "vtx", &value) == ESP_OK)
    {
        m_config.vtxBand = value >> 24;
        m_config.vtxChannel = value >> 16;
        m_config.vtxPower = value >> 8;
        m_config.vtxPitmode = value;
    }

    // fanthresh (v5)
    if (nvs_get_u8(handle, "fanthresh", &value8) == ESP_OK)
        m_config.powerFanThreshold = value8;

    // Both of these were added to config v5 without incrementing the version
    if (nvs_get_u32(handle, "fan", &value) == ESP_OK)
        m_config.fanMode = value;
    if (nvs_get_u32(handle, "motion", &value) == ESP_OK)
        m_config.motionMode = value;

    if (version >= 6)
    {
        // dvr (v6)
        if (nvs_get_u8(handle, "dvraux", &value8) == ESP_OK)
            m_config.dvrAux = value8;
        if (nvs_get_u8(handle, "dvrstartdelay", &value8) == ESP_OK)
            m_config.dvrStartDelay = value8;
        if (nvs_get_u8(handle, "dvrstopdelay", &value8) == ESP_OK)
            m_config.dvrStopDelay = value8;
    }
    else
    {
        // Need to write the dvr defaults
        m_modified |= EVENT_CONFIG_MAIN_CHANGED;
    }

    if (version >= 7) {
        // load button actions
        if (nvs_get_u32(handle, "button1", &value) == ESP_OK)
            m_config.buttonColors[0].raw = value;
        if (nvs_get_u32(handle, "button2", &value) == ESP_OK)
            m_config.buttonColors[1].raw = value;
        // backpackdisable was actually added after 7, but if not found will default to 0 (enabled)
        if (nvs_get_u8(handle, "backpackdisable", &value8) == ESP_OK)
            m_config.backpackDisable = value8;
        if (nvs_get_u8(handle, "backpacktlmen", &value8) == ESP_OK)
            m_config.backpackTlmMode = value8;
    }

    for(unsigned i=0; i<CONFIG_TX_MODEL_CNT; i++)
    {
        uint64_t model_value;
        uint32_t old_value;
        char model[10] = "model";
        itoa(i, model+5, 10);
        if (version < TX_CONFIG_VERSION && nvs_get_u32(handle, model, &old_value) == ESP_OK)
        {
            // Chaining update, last calls nvs_set_u64, all others set `value`
            if (version == 6)
            {
                // Upgrade v6 to v7
                v6_model_config_t v6model;
                U32_to_Model(old_value, &v6model);
                v7_model_config_t v7Model;
                ModelV6toV7(&v6model, &v7Model);
                old_value = Model_to_U32(&v7Model);
            }

            if (version <= 7)
            {
                // Upgrade v7 to v8
                v7_model_config_t v7model;
                U32_to_Model(old_value, &v7model);
                v8_model_config_t v8Model;
                ModelV7toV8(&v7model, &v8Model);
                old_value = Model_to_U32(&v8Model);
            }

            if (version <= 8)
            {
                v8_model_config_t v8model;
                U32_to_Model(old_value, &v8model);
                model_config_t * const newModel = &m_config.model_config[i];
                ModelV8toV9(&v8model, newModel);
                nvs_set_u64(handle, model, Model_to_U64(newModel));
            }
        }
        else if (version == TX_CONFIG_VERSION && nvs_get_u64(handle, model, &model_value) == ESP_OK)
        {
            U64_to_Model(model_value, &m_config.model_config[i]);
        }
    } // for each model

    if (version != TX_CONFIG_VERSION)
    {
        m_modified |= EVENT_CONFIG_VERSION_CHANGED;
        Commit();
    }
}
#else  // ESP8266
void TxConfig::Load()
{
    EEPROM.begin(1024);
    m_modified = 0;
    EEPROM.get(0, m_config);

    uint32_t version = 0;
    if ((m_config.version & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC)
        version = m_config.version & ~CONFIG_MAGIC_MASK;
    DBGLN("Config version %u", version);

    // If version is current, all done
    if (version == TX_CONFIG_VERSION)
        return;

    // Can't upgrade from version <5, or when flashing a previous version, just use defaults.
    if (version < 5 || version > TX_CONFIG_VERSION)
    {
        SetDefaults(true);
        return;
    }

    // Upgrade EEPROM, starting with defaults
    SetDefaults(false);

    if (version == 5)
    {
        UpgradeEepromV5ToV6();
        version = 6;
    }

    if (version == 6)
    {
        UpgradeEepromV6ToV7();
        version = 7;
    }

    if (version == 7)
    {
        UpgradeEepromV7ToV8();
        version = 8;
    }

    if (version == 8)
    {
        UpgradeEepromV8ToV9();
    }
}

void TxConfig::UpgradeEepromV5ToV6()
{
    v5_tx_config_t v5Config;
    v6_tx_config_t v6Config = { 0 }; // default the new fields to 0

    // Populate the prev version struct from eeprom
    EEPROM.get(0, v5Config);

    // Copy prev values to current config struct
    // This only workse because v5 and v6 are the same up to the new fields
    // which have already been set to 0
    memcpy(&v6Config, &v5Config, sizeof(v5Config));
    v6Config.version = 6U | TX_CONFIG_MAGIC;
    EEPROM.put(0, v6Config);
    EEPROM.commit();
}

void TxConfig::UpgradeEepromV6ToV7()
{
    v6_tx_config_t v6Config;
    v7_tx_config_t v7Config = { 0 }; // default the new fields to 0

    // Populate the prev version struct from eeprom
    EEPROM.get(0, v6Config);

    // Manual field copying as some fields have moved
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

    for (unsigned i=0; i<CONFIG_TX_MODEL_CNT; i++)
    {
        ModelV6toV7(&v6Config.model_config[i], &v7Config.model_config[i]);
    }

    m_modified = ALL_CHANGED;

    // Full Commit now
    m_config.version = 7U | TX_CONFIG_MAGIC;
    EEPROM.put(0, v7Config);
    EEPROM.commit();
}

void TxConfig::UpgradeEepromV7ToV8()
{
    v7_tx_config_t v7Config;
    v8_tx_config_t v8Config = {};
    EEPROM.get(0, v7Config);

    // Manual field copying as some fields were removed
    #define LAZY(member) v8Config.member = v7Config.member
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

    for (unsigned i=0; i<CONFIG_TX_MODEL_CNT; i++)
    {
        ModelV7toV8(&v7Config.model_config[i], &v8Config.model_config[i]);
    }

    m_modified = ALL_CHANGED;

    // Full Commit now
    m_config.version = 8U | TX_CONFIG_MAGIC;
    EEPROM.put(0, v8Config);
    EEPROM.commit();
}

void TxConfig::UpgradeEepromV8ToV9()
{
    v8_tx_config_t v8Config = {};
    EEPROM.get(0, v8Config);

    // Manual field copying as some fields were removed
#define LAZY(member) m_config.member = v8Config.member
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

    for (unsigned i=0; i<CONFIG_TX_MODEL_CNT; i++)
    {
        ModelV8toV9(&v8Config.model_config[i], &m_config.model_config[i]);
    }

    m_modified = ALL_CHANGED;

    // Full Commit now
    m_config.version = 9U | TX_CONFIG_MAGIC;
    Commit();
}
#endif

uint32_t
TxConfig::Commit()
{
    if (!m_modified)
    {
        DBGLN("No changes");
        // No changes
        return 0;
    }
#if defined(PLATFORM_ESP32)
    // Write parts to NVS
    if (m_modified & EVENT_CONFIG_MODEL_CHANGED)
    {
        uint64_t value = Model_to_U64(m_model);
        char model[10] = "model";
        itoa(m_modelId, model+5, 10);
        nvs_set_u64(handle, model, value);
    }
    if (m_modified & EVENT_CONFIG_VTX_CHANGED)
    {
        uint32_t value =
            m_config.vtxBand << 24 |
            m_config.vtxChannel << 16 |
            m_config.vtxPower << 8 |
            m_config.vtxPitmode;
        nvs_set_u32(handle, "vtx", value);
    }
    if (m_modified & EVENT_CONFIG_FAN_CHANGED)
    {
        uint32_t value = m_config.fanMode;
        nvs_set_u32(handle, "fan", value);
        nvs_set_u8(handle, "fanthresh", m_config.powerFanThreshold);
    }
    if (m_modified & EVENT_CONFIG_MOTION_CHANGED)
    {
        uint32_t value = m_config.motionMode;
        nvs_set_u32(handle, "motion", value);
    }
    if (m_modified & EVENT_CONFIG_MAIN_CHANGED)
    {
        nvs_set_u8(handle, "backpackdisable", m_config.backpackDisable);
        nvs_set_u8(handle, "backpacktlmen", m_config.backpackTlmMode);
        nvs_set_u8(handle, "dvraux", m_config.dvrAux);
        nvs_set_u8(handle, "dvrstartdelay", m_config.dvrStartDelay);
        nvs_set_u8(handle, "dvrstopdelay", m_config.dvrStopDelay);
    }
    if (m_modified & EVENT_CONFIG_BUTTON_CHANGED)
    {
        nvs_set_u32(handle, "button1", m_config.buttonColors[0].raw);
        nvs_set_u32(handle, "button2", m_config.buttonColors[1].raw);
    }
    if (m_modified & EVENT_CONFIG_VERSION_CHANGED)
    {
        nvs_set_u32(handle, "tx_version", m_config.version);
    }
    nvs_commit(handle);
#else
    // Write the struct to eeprom
    EEPROM.put(0, m_config);
    EEPROM.commit();
#endif
    uint32_t changes = m_modified;
    m_modified = 0;
    return changes;
}

// Setters
void
TxConfig::SetRate(uint8_t rate)
{
    if (GetRate() != rate)
    {
        m_model->rate = rate;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetTlm(uint8_t tlm)
{
    if (GetTlm() != tlm)
    {
        m_model->tlm = tlm;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetPower(uint8_t power)
{
    if (GetPower() != power)
    {
        m_model->power = power;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetDynamicPower(bool dynamicPower)
{
    if (GetDynamicPower() != dynamicPower)
    {
        m_model->dynamicPower = dynamicPower;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetBoostChannel(uint8_t boostChannel)
{
    if (GetBoostChannel() != boostChannel)
    {
        m_model->boostChannel = boostChannel;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetSwitchMode(uint8_t switchMode)
{
    if (GetSwitchMode() != switchMode)
    {
        m_model->switchMode = switchMode;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetAntennaMode(uint8_t txAntenna)
{
    if (GetAntennaMode() != txAntenna)
    {
        m_model->txAntenna = txAntenna;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetLinkMode(uint8_t linkMode)
{
    if (GetLinkMode() != linkMode)
    {
        m_model->linkMode = linkMode;

        if (linkMode == TX_MAVLINK_MODE)
        {
            m_model->tlm = TLM_RATIO_1_2;
            m_model->switchMode = smHybridOr16ch; // Force Hybrid / 16ch/2 switch modes for mavlink
        }
        m_modified |= EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
TxConfig::SetModelMatch(bool modelMatch)
{
    if (GetModelMatch() != modelMatch)
    {
        m_model->modelMatch = modelMatch;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetRelayEnabled(bool relayEnabled)
{
    if (GetRelayEnabled() != relayEnabled)
    {
        m_model->relay = relayEnabled;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetVtxBand(uint8_t vtxBand)
{
    if (m_config.vtxBand != vtxBand)
    {
        m_config.vtxBand = vtxBand;
        m_modified |= EVENT_CONFIG_VTX_CHANGED;
    }
}

void
TxConfig::SetVtxChannel(uint8_t vtxChannel)
{
    if (m_config.vtxChannel != vtxChannel)
    {
        m_config.vtxChannel = vtxChannel;
        m_modified |= EVENT_CONFIG_VTX_CHANGED;
    }
}

void
TxConfig::SetVtxPower(uint8_t vtxPower)
{
    if (m_config.vtxPower != vtxPower)
    {
        m_config.vtxPower = vtxPower;
        m_modified |= EVENT_CONFIG_VTX_CHANGED;
    }
}

void
TxConfig::SetVtxPitmode(uint8_t vtxPitmode)
{
    if (m_config.vtxPitmode != vtxPitmode)
    {
        m_config.vtxPitmode = vtxPitmode;
        m_modified |= EVENT_CONFIG_VTX_CHANGED;
    }
}

void
TxConfig::SetPowerFanThreshold(uint8_t powerFanThreshold)
{
    if (m_config.powerFanThreshold != powerFanThreshold)
    {
        m_config.powerFanThreshold = powerFanThreshold;
        m_modified |= EVENT_CONFIG_FAN_CHANGED;
    }
}

void
TxConfig::SetFanMode(uint8_t fanMode)
{
    if (m_config.fanMode != fanMode)
    {
        m_config.fanMode = fanMode;
        m_modified |= EVENT_CONFIG_FAN_CHANGED;
    }
}

void
TxConfig::SetMotionMode(uint8_t motionMode)
{
    if (m_config.motionMode != motionMode)
    {
        m_config.motionMode = motionMode;
        m_modified |= EVENT_CONFIG_MOTION_CHANGED;
    }
}

void
TxConfig::SetDvrAux(uint8_t dvrAux)
{
    if (GetDvrAux() != dvrAux)
    {
        m_config.dvrAux = dvrAux;
        m_modified |= EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
TxConfig::SetDvrStartDelay(uint8_t dvrStartDelay)
{
    if (GetDvrStartDelay() != dvrStartDelay)
    {
        m_config.dvrStartDelay = dvrStartDelay;
        m_modified |= EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
TxConfig::SetDvrStopDelay(uint8_t dvrStopDelay)
{
    if (GetDvrStopDelay() != dvrStopDelay)
    {
        m_config.dvrStopDelay = dvrStopDelay;
        m_modified |= EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
TxConfig::SetBackpackDisable(bool backpackDisable)
{
    if (m_config.backpackDisable != backpackDisable)
    {
        m_config.backpackDisable = backpackDisable;
        m_modified |= EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
TxConfig::SetBackpackTlmMode(uint8_t mode)
{
    if (m_config.backpackTlmMode != mode)
    {
        m_config.backpackTlmMode = mode;
        m_modified |= EVENT_CONFIG_MAIN_CHANGED;
    }
}

void
TxConfig::SetButtonActions(uint8_t button, tx_button_color_t *action)
{
    if (m_config.buttonColors[button].raw != action->raw) {
        m_config.buttonColors[button].raw = action->raw;
        m_modified |= EVENT_CONFIG_BUTTON_CHANGED;
    }
}

void
TxConfig::SetPTRStartChannel(uint8_t ptrStartChannel)
{
    if (ptrStartChannel != m_model->ptrStartChannel) {
        m_model->ptrStartChannel = ptrStartChannel;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetPTREnableChannel(uint8_t ptrEnableChannel)
{
    if (ptrEnableChannel != m_model->ptrEnableChannel) {
        m_model->ptrEnableChannel = ptrEnableChannel;
        m_modified |= EVENT_CONFIG_MODEL_CHANGED;
    }
}

void
TxConfig::SetDefaults(bool commit)
{
    // Reset everything to 0/false and then just set anything that zero is not appropriate
    memset(&m_config, 0, sizeof(m_config));

    m_config.version = TX_CONFIG_VERSION | TX_CONFIG_MAGIC;
    m_config.powerFanThreshold = PWR_250mW;
    m_modified = ALL_CHANGED;

    // Set defaults for button 1
    tx_button_color_t default_actions1 = {
        .val = {
            .color = 226,   // R:255 G:0 B:182
            .actions = {
                {false, 2, ACTION_BIND},
                {true, 0, ACTION_INCREASE_POWER}
            }
        }
    };
    m_config.buttonColors[0].raw = default_actions1.raw;

    // Set defaults for button 2
    tx_button_color_t default_actions2 = {
        .val = {
            .color = 3,     // R:0 G:0 B:255
            .actions = {
                {false, 1, ACTION_GOTO_VTX_CHANNEL},
                {true, 0, ACTION_SEND_VTX}
            }
        }
    };
    m_config.buttonColors[1].raw = default_actions2.raw;

    for (unsigned i=0; i<CONFIG_TX_MODEL_CNT; i++)
    {
        SetModelId(i);
        #if defined(RADIO_SX127X)
            SetRate(enumRatetoIndex(RATE_LORA_900_200HZ));
        #elif defined(RADIO_LR1121)
            SetRate(enumRatetoIndex(POWER_OUTPUT_VALUES_COUNT == 0 ? RATE_LORA_2G4_250HZ : RATE_LORA_900_200HZ));
        #elif defined(RADIO_SX128X)
            SetRate(enumRatetoIndex(RATE_LORA_2G4_250HZ));
        #endif
        SetPower(POWERMGNT::getDefaultPower());
#if defined(PLATFORM_ESP32)
        // ESP32 nvs needs to commit every model
        if (commit)
        {
            m_modified |= EVENT_CONFIG_MODEL_CHANGED;
            Commit();
        }
#endif
    }

#if !defined(PLATFORM_ESP32)
    // ESP8266 just needs one commit
    if (commit)
    {
        Commit();
    }
#endif

    SetModelId(0);
    m_modified = 0;
}

/**
 * Sets ModelId used for subsequent per-model config gets
 * Returns: true if the model has changed
 **/
bool
TxConfig::SetModelId(uint8_t modelId)
{
    model_config_t *newModel = &m_config.model_config[modelId];
    if (newModel != m_model)
    {
        m_model = newModel;
        m_modelId = modelId;
        return true;
    }

    return false;
}
#endif
