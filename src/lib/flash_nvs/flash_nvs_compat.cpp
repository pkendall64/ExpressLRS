#include "flash_nvs_compat.h"

#if !defined(PLATFORM_ESP32)

namespace
{
constexpr nvs_handle FLASH_NVS_DEFAULT_HANDLE = 1U;

FlashNVS g_flashNvs;
flash_nvs_config_t g_flashNvsConfig{};
bool g_flashNvsConfigured = false;
bool g_flashNvsInitialized = false;


bool isValidHandle(nvs_handle handle)
{
    return g_flashNvsInitialized && handle == FLASH_NVS_DEFAULT_HANDLE;
}
}

esp_err_t flash_nvs_set_config(const flash_nvs_config_t *config)
{
    if (!config)
        return ESP_FAIL;
    g_flashNvsConfig = *config;
    g_flashNvsConfigured = true;
    g_flashNvsInitialized = false;
    return ESP_OK;
}

esp_err_t nvs_flash_init(void)
{
    if (!g_flashNvsConfigured)
        return ESP_FAIL;
    esp_err_t result = g_flashNvs.Begin(g_flashNvsConfig);
    g_flashNvsInitialized = (result == ESP_OK);
    return result;
}

esp_err_t nvs_flash_erase(void)
{
    if (!g_flashNvsConfigured)
        return ESP_FAIL;
    esp_err_t result = g_flashNvs.EraseAll();
    g_flashNvsInitialized = (result == ESP_OK);
    return result;
}

esp_err_t nvs_open(const char *name, nvs_open_mode_t open_mode, nvs_handle *out_handle)
{
    (void)name;
    (void)open_mode;
    if (!out_handle || !g_flashNvsInitialized)
        return ESP_FAIL;
    *out_handle = FLASH_NVS_DEFAULT_HANDLE;
    return ESP_OK;
}

void nvs_close(nvs_handle handle)
{
    (void)handle;
}

esp_err_t nvs_get_u8(nvs_handle handle, nvs_key_t key, uint8_t *out_value)
{
    if (!isValidHandle(handle) || !out_value)
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.GetU8(key, out_value);
}

esp_err_t nvs_get_u32(nvs_handle handle, nvs_key_t key, uint32_t *out_value)
{
    if (!isValidHandle(handle) || !out_value)
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.GetU32(key, out_value);
}

esp_err_t nvs_get_blob(nvs_handle handle, nvs_key_t key, void *out_value, size_t *length)
{
    if (!isValidHandle(handle) || !length)
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.GetBlob(key, out_value, length);
}

esp_err_t nvs_set_u8(nvs_handle handle, nvs_key_t key, uint8_t value)
{
    if (!isValidHandle(handle))
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.SetU8(key, value);
}

esp_err_t nvs_set_u32(nvs_handle handle, nvs_key_t key, uint32_t value)
{
    if (!isValidHandle(handle))
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.SetU32(key, value);
}

esp_err_t nvs_set_blob(nvs_handle handle, nvs_key_t key, const void *value, size_t length)
{
    if (!isValidHandle(handle))
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.SetBlob(key, value, length);
}

esp_err_t nvs_commit(nvs_handle handle)
{
    if (!isValidHandle(handle))
        return ESP_ERR_NVS_INVALID_HANDLE;
    return g_flashNvs.Commit();
}

#endif
