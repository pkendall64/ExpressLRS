#pragma once

#include "config.h"

#if defined(TARGET_TX)

#define ALL_CHANGED         (EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_VTX_CHANGED | EVENT_CONFIG_MAIN_CHANGED | EVENT_CONFIG_FAN_CHANGED | EVENT_CONFIG_MOTION_CHANGED | EVENT_CONFIG_BUTTON_CHANGED | EVENT_CONFIG_VERSION_CHANGED)

// Really awful but safe(?) type punning of model_config_t/v6_model_config_t to and from uint32_t
template<class T> static inline void U32_to_Model(uint32_t const u32, T * const model)
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

template<class T> static inline uint32_t Model_to_U32(T const * const model)
{
    // clear the entire union because the assignment will only fill sizeof(T)
    union {
        union {
            T model;
            uint8_t padding[sizeof(uint32_t)-sizeof(T)];
        } val;
        uint32_t u32;
    } converter = { 0 };

    converter.val.model = *model;
    return converter.u32;
}

#if defined(PLATFORM_ESP32)
typedef const char *tx_nvs_key_t;
static constexpr tx_nvs_key_t TX_NVS_KEY_VERSION = "tx_version";
static constexpr tx_nvs_key_t TX_NVS_KEY_VTX = "vtx";
static constexpr tx_nvs_key_t TX_NVS_KEY_FANTHRESH = "fanthresh";
static constexpr tx_nvs_key_t TX_NVS_KEY_FAN = "fan";
static constexpr tx_nvs_key_t TX_NVS_KEY_MOTION = "motion";
static constexpr tx_nvs_key_t TX_NVS_KEY_DVRAUX = "dvraux";
static constexpr tx_nvs_key_t TX_NVS_KEY_DVRSTARTDELAY = "dvrstartdelay";
static constexpr tx_nvs_key_t TX_NVS_KEY_DVRSTOPDELAY = "dvrstopdelay";
static constexpr tx_nvs_key_t TX_NVS_KEY_BUTTON1 = "button1";
static constexpr tx_nvs_key_t TX_NVS_KEY_BUTTON2 = "button2";
static constexpr tx_nvs_key_t TX_NVS_KEY_BACKPACKDISABLE = "backpackdisable";
static constexpr tx_nvs_key_t TX_NVS_KEY_BACKPACKTLMEN = "backpacktlmen";
#define TX_MODEL_KEY_DECLARE(name, idx) char name[10] = "model"; itoa((idx), name + 5, 10)
#define TX_MODEL_KEY(name) name
#else
typedef nvs_key_t tx_nvs_key_t;
enum : tx_nvs_key_t {
    TX_NVS_KEY_VERSION = 1,
    TX_NVS_KEY_VTX = 2,
    TX_NVS_KEY_FANTHRESH = 3,
    TX_NVS_KEY_FAN = 4,
    TX_NVS_KEY_MOTION = 5,
    TX_NVS_KEY_DVRAUX = 6,
    TX_NVS_KEY_DVRSTARTDELAY = 7,
    TX_NVS_KEY_DVRSTOPDELAY = 8,
    TX_NVS_KEY_BUTTON1 = 9,
    TX_NVS_KEY_BUTTON2 = 10,
    TX_NVS_KEY_BACKPACKDISABLE = 11,
    TX_NVS_KEY_BACKPACKTLMEN = 12,
    TX_NVS_KEY_MODEL_BASE = 0x100,
};
#define TX_MODEL_KEY_DECLARE(name, idx) const tx_nvs_key_t name = TX_NVS_KEY_MODEL_BASE + static_cast<tx_nvs_key_t>(idx)
#define TX_MODEL_KEY(name) name
#endif

#endif
