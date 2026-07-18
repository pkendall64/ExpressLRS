#pragma once

#include "config.h"

#if defined(TARGET_RX)

#if defined(PLATFORM_ESP32)
typedef const char *rx_nvs_key_t;
static constexpr rx_nvs_key_t RX_NVS_KEY_VERSION = "rx_version";
static constexpr rx_nvs_key_t RX_NVS_KEY_IDENTITY = "rx_identity";
static constexpr rx_nvs_key_t RX_NVS_KEY_POWER_ON_COUNT = "rx_poc";
static constexpr rx_nvs_key_t RX_NVS_KEY_MAIN = "rx_main";
static constexpr rx_nvs_key_t RX_NVS_KEY_SERIAL = "rx_serial";
#define RX_PWM_KEY_DECLARE(name, channel) char name[8] = "pwm"; itoa((channel), name + 3, 10)
#define RX_PWM_KEY(name) name
#elif defined(PLATFORM_ESP8266)
typedef nvs_key_t rx_nvs_key_t;
enum : nvs_key_t {
    RX_NVS_KEY_VERSION = 1,
    RX_NVS_KEY_IDENTITY = 2,
    RX_NVS_KEY_POWER_ON_COUNT = 3,
    RX_NVS_KEY_MAIN = 4,
    RX_NVS_KEY_SERIAL = 5,
    RX_NVS_KEY_PWM_BASE = 0x100,
};
static inline uint32_t RxPwmKey(uint8_t channel)
{
    return RX_NVS_KEY_PWM_BASE + channel;
}
#define RX_PWM_KEY_DECLARE(name, channel) const rx_nvs_key_t name = RxPwmKey(channel)
#define RX_PWM_KEY(name) name
#endif

typedef struct __attribute__((packed)) {
    uint8_t uid[UID_LEN];
    uint32_t flash_discriminator;
    uint8_t bindStorage;
} rx_identity_storage_t;

typedef struct __attribute__((packed)) {
    uint8_t power;
    uint8_t antennaMode;
    uint8_t antennaGroup;
    uint8_t forceTlmOff;
    uint8_t rateInitialIdx;
    uint8_t modelId;
    uint8_t failsafeMode;
    uint8_t teamraceChannel;
    uint8_t teamracePosition;
    uint8_t teamracePitMode;
    uint8_t targetSysId;
    uint8_t sourceSysId;
} rx_main_storage_t;

typedef struct __attribute__((packed)) {
    uint8_t serialProtocol;
    uint8_t serial1Protocol;
} rx_serial_storage_t;

#define ALL_CHANGED (EVENT_CONFIG_UID_CHANGED | EVENT_CONFIG_POWER_COUNT_CHANGED | EVENT_CONFIG_MODEL_CHANGED | EVENT_CONFIG_MAIN_CHANGED | EVENT_CONFIG_SERIAL_CHANGE | EVENT_CONFIG_PWM_CHANGE | EVENT_CONFIG_VERSION_CHANGED)

#endif
