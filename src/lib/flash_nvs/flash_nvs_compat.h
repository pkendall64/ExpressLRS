#pragma once

#include <stdint.h>
#include <stddef.h>

#include "flash_nvs.h"

#if !defined(PLATFORM_ESP32)

typedef int32_t esp_err_t;
typedef uint32_t nvs_handle;
typedef uint32_t nvs_key_t;
typedef uint8_t nvs_open_mode_t;

enum
{
    NVS_READONLY = 0,
    NVS_READWRITE = 1,
};

enum
{
    ESP_OK = FLASH_NVS_OK,
    ESP_FAIL = FLASH_NVS_ERR_FLASH,
    ESP_ERR_NVS_NOT_FOUND = FLASH_NVS_ERR_NOT_FOUND,
    ESP_ERR_NVS_INVALID_HANDLE = FLASH_NVS_ERR_INVALID_ARG,
    ESP_ERR_NVS_INVALID_LENGTH = FLASH_NVS_ERR_DATA_TOO_LONG,
    ESP_ERR_NVS_NO_FREE_PAGES = FLASH_NVS_ERR_NO_SPACE,
    ESP_ERR_NVS_NEW_VERSION_FOUND = FLASH_NVS_ERR_CORRUPT,
};

esp_err_t flash_nvs_set_config(const flash_nvs_config_t *config);

#if defined(PLATFORM_ESP8266) && !defined(TARGET_NATIVE)
esp_err_t flash_nvs_set_esp8266_default_config(void);
bool flash_nvs_has_store(void);
#endif

esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);

esp_err_t nvs_open(const char *name, nvs_open_mode_t open_mode, nvs_handle *out_handle);
void nvs_close(nvs_handle handle);

esp_err_t nvs_get_u8(nvs_handle handle, nvs_key_t key, uint8_t *out_value);
esp_err_t nvs_get_u32(nvs_handle handle, nvs_key_t key, uint32_t *out_value);
esp_err_t nvs_get_blob(nvs_handle handle, nvs_key_t key, void *out_value, size_t *length);

esp_err_t nvs_set_u8(nvs_handle handle, nvs_key_t key, uint8_t value);
esp_err_t nvs_set_u32(nvs_handle handle, nvs_key_t key, uint32_t value);
esp_err_t nvs_set_blob(nvs_handle handle, nvs_key_t key, const void *value, size_t length);

esp_err_t nvs_commit(nvs_handle handle);

#endif
