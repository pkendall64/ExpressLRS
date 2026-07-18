#include "tx_config_internal.h"

#include "POWERMGNT.h"
#include "helpers.h"
#include "logging.h"

#if defined(TARGET_TX)

TxConfig::TxConfig() :
    BindphraseConfigurable(), m_model(m_config.model_config)
{
}

void TxConfig::Load()
{
    m_modified = 0;

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
    {
        ERRLN("TxConfig NVS init failed");
        SetDefaults(false);
        return;
    }
    if (nvs_open("ELRS", NVS_READWRITE, &handle) != ESP_OK)
    {
        ERRLN("TxConfig NVS open failed");
        SetDefaults(false);
        return;
    }

    uint32_t version = 0;
    if (nvs_get_u32(handle, TX_NVS_KEY_VERSION, &version) == ESP_OK && ((version & CONFIG_MAGIC_MASK) == TX_CONFIG_MAGIC))
        version &= ~CONFIG_MAGIC_MASK;

    DBGLN("Config version %u", version);

    if (version == TX_CONFIG_VERSION)
    {
        SetDefaults(false);

        uint32_t value;
        uint8_t value8;

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
        if (nvs_get_u8(handle, TX_NVS_KEY_DVRAUX, &value8) == ESP_OK)
            m_config.dvrAux = value8;
        if (nvs_get_u8(handle, TX_NVS_KEY_DVRSTARTDELAY, &value8) == ESP_OK)
            m_config.dvrStartDelay = value8;
        if (nvs_get_u8(handle, TX_NVS_KEY_DVRSTOPDELAY, &value8) == ESP_OK)
            m_config.dvrStopDelay = value8;
        if (nvs_get_u32(handle, TX_NVS_KEY_BUTTON1, &value) == ESP_OK)
            m_config.buttonColors[0].raw = value;
        if (nvs_get_u32(handle, TX_NVS_KEY_BUTTON2, &value) == ESP_OK)
            m_config.buttonColors[1].raw = value;
        if (nvs_get_u8(handle, TX_NVS_KEY_BACKPACKDISABLE, &value8) == ESP_OK)
            m_config.backpackDisable = value8;
        if (nvs_get_u8(handle, TX_NVS_KEY_BACKPACKTLMEN, &value8) == ESP_OK)
            m_config.backpackTlmMode = value8;

        for (unsigned i = 0; i < CONFIG_TX_MODEL_CNT; i++)
        {
            TX_MODEL_KEY_DECLARE(modelKey, i);
            if (nvs_get_u32(handle, TX_MODEL_KEY(modelKey), &value) == ESP_OK)
            {
                U32_to_Model(value, &m_config.model_config[i]);

                if (!isSupportedRFRate(m_config.model_config[i].rate))
                {
                    // validate the currently selected rate is supported by the hardware and choose an appropriate default if not
                    m_config.model_config[i].rate = enumRatetoIndexSafe(POWER_OUTPUT_VALUES_COUNT == 0 ? RATE_LORA_2G4_250HZ : RATE_LORA_900_200HZ);
                    nvs_set_u32(handle, TX_MODEL_KEY(modelKey), Model_to_U32(&m_config.model_config[i]));
                }
            }
        }
        return;
    }

    if (MigrateLegacyConfig())
    {
        Commit();
        return;
    }

    SetDefaults(true);
}


uint32_t
TxConfig::Commit()
{
    if (!m_modified)
    {
        DBGLN("No changes");
        // No changes
        return 0;
    }
    TX_MODEL_KEY_DECLARE(modelKey, m_modelId);
    if (m_modified & EVENT_CONFIG_MODEL_CHANGED)
    {
        nvs_set_u32(handle, TX_MODEL_KEY(modelKey), Model_to_U32(m_model));
    }
    if (m_modified & EVENT_CONFIG_VTX_CHANGED)
    {
        uint32_t value =
            m_config.vtxBand << 24 |
            m_config.vtxChannel << 16 |
            m_config.vtxPower << 8 |
            m_config.vtxPitmode;
        nvs_set_u32(handle, TX_NVS_KEY_VTX, value);
    }
    if (m_modified & EVENT_CONFIG_FAN_CHANGED)
    {
        nvs_set_u32(handle, TX_NVS_KEY_FAN, m_config.fanMode);
        nvs_set_u8(handle, TX_NVS_KEY_FANTHRESH, m_config.powerFanThreshold);
    }
    if (m_modified & EVENT_CONFIG_MOTION_CHANGED)
    {
        nvs_set_u32(handle, TX_NVS_KEY_MOTION, m_config.motionMode);
    }
    if (m_modified & EVENT_CONFIG_MAIN_CHANGED)
    {
        nvs_set_u8(handle, TX_NVS_KEY_BACKPACKDISABLE, m_config.backpackDisable);
        nvs_set_u8(handle, TX_NVS_KEY_BACKPACKTLMEN, m_config.backpackTlmMode);
        nvs_set_u8(handle, TX_NVS_KEY_DVRAUX, m_config.dvrAux);
        nvs_set_u8(handle, TX_NVS_KEY_DVRSTARTDELAY, m_config.dvrStartDelay);
        nvs_set_u8(handle, TX_NVS_KEY_DVRSTOPDELAY, m_config.dvrStopDelay);
    }
    if (m_modified & EVENT_CONFIG_BUTTON_CHANGED)
    {
        nvs_set_u32(handle, TX_NVS_KEY_BUTTON1, m_config.buttonColors[0].raw);
        nvs_set_u32(handle, TX_NVS_KEY_BUTTON2, m_config.buttonColors[1].raw);
    }
    if (m_modified & EVENT_CONFIG_VERSION_CHANGED)
    {
        nvs_set_u32(handle, TX_NVS_KEY_VERSION, m_config.version);
    }
    nvs_commit(handle);
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
            SetRate(enumRatetoIndexSafe(RATE_LORA_900_200HZ));
        #elif defined(RADIO_LR1121)
            SetRate(enumRatetoIndexSafe(POWER_OUTPUT_VALUES_COUNT == 0 ? RATE_LORA_2G4_250HZ : RATE_LORA_900_200HZ));
        #elif defined(RADIO_SX128X)
            SetRate(enumRatetoIndexSafe(RATE_LORA_2G4_250HZ));
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

void TxConfig::SetUID(uint8_t uid[UID_LEN])
{
    // The UID is only stored in the options.json, not in nvs/eeprom like on the RX
    // Emulate the setting as a config setting to have the same access method as the RX
    firmwareOptions.hasUID = OtaUidIsBound(uid);
    memcpy(firmwareOptions.uid, uid, UID_LEN);
    saveOptions();
}

#endif

