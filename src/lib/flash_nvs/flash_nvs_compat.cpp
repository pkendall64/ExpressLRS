#include "flash_nvs_compat.h"

#if defined(PLATFORM_ESP8266) && !defined(TARGET_NATIVE)
#include <Arduino.h>

extern "C" uint32_t _FS_start;
extern "C" uint32_t _EEPROM_start;
#endif

#if !defined(PLATFORM_ESP32)

namespace
{
constexpr nvs_handle FLASH_NVS_DEFAULT_HANDLE = 1U;

FlashNVS g_flashNvs;
flash_nvs_config_t g_flashNvsConfig{};
bool g_flashNvsConfigured = false;
bool g_flashNvsInitialized = false;

struct FlashNvsHeader
{
    uint32_t magic;
    uint32_t version;
    uint32_t generation;
    uint32_t crc;
};

uint32_t flashNvsHeaderCrc(uint32_t magic, uint32_t version, uint32_t generation)
{
    uint32_t crc = 2166136261UL;
    const uint32_t fields[3] = {magic, version, generation};
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(fields);
    for (size_t i = 0; i < sizeof(fields); ++i)
    {
        crc ^= bytes[i];
        crc *= 16777619UL;
    }
    return crc;
}


bool isValidHandle(nvs_handle handle)
{
    return g_flashNvsInitialized && handle == FLASH_NVS_DEFAULT_HANDLE;
}

#if defined(PLATFORM_ESP8266) && !defined(TARGET_NATIVE)
constexpr uint32_t FLASH_NVS_ESP8266_BASE = 0x40200000UL;

bool flashNvsEsp8266Read(void *context, uint32_t address, void *dst, size_t len)
{
    (void)context;
    uint8_t *out = static_cast<uint8_t *>(dst);
    uint32_t alignedAddress = address & ~uint32_t(0x3U);
    size_t skip = address - alignedAddress;
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

bool flashNvsEsp8266Write(void *context, uint32_t address, const void *src, size_t len)
{
    (void)context;
    const uint8_t *input = static_cast<const uint8_t *>(src);
    if ((address & 0x3U) != 0U || (len & 0x3U) != 0U)
        return false;

    uint32_t words[8];
    while (len > 0)
    {
        size_t chunk = len > sizeof(words) ? sizeof(words) : len;
        memcpy(words, input, chunk);
        if (!ESP.flashWrite(address, words, chunk))
            return false;
        input += chunk;
        address += static_cast<uint32_t>(chunk);
        len -= chunk;
    }
    return true;
}


bool flashNvsEsp8266Erase(void *context, uint32_t address, size_t len)
{
    (void)context;
    if ((address & (SPI_FLASH_SEC_SIZE - 1U)) != 0U || len != SPI_FLASH_SEC_SIZE)
        return false;
    return ESP.flashEraseSector(address / SPI_FLASH_SEC_SIZE);
}
#endif
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

#if defined(PLATFORM_ESP8266) && !defined(TARGET_NATIVE)
esp_err_t flash_nvs_set_esp8266_default_config(void)
{
    flash_nvs_config_t config{};
    config.flash.context = nullptr;
    config.flash.read = flashNvsEsp8266Read;
    config.flash.write = flashNvsEsp8266Write;
    config.flash.erase = flashNvsEsp8266Erase;
    config.sector0Address = reinterpret_cast<uint32_t>(&_FS_start) - 0x1000U - FLASH_NVS_ESP8266_BASE;
    config.sector1Address = reinterpret_cast<uint32_t>(&_EEPROM_start) - FLASH_NVS_ESP8266_BASE;
    config.sectorSize = SPI_FLASH_SEC_SIZE;
    return flash_nvs_set_config(&config);
}

bool flash_nvs_has_store(void)
{
    if (!g_flashNvsConfigured)
        return false;

    FlashNvsHeader header{};
    for (uint32_t address : {g_flashNvsConfig.sector0Address, g_flashNvsConfig.sector1Address})
    {
        if (!g_flashNvsConfig.flash.read(g_flashNvsConfig.flash.context, address, &header, sizeof(header)))
            continue;
        if (header.magic != FLASH_NVS_SECTOR_MAGIC || header.version != FLASH_NVS_VERSION)
            continue;
        if (header.crc == flashNvsHeaderCrc(header.magic, header.version, header.generation))
            return true;
    }
    return false;
}
#endif

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
