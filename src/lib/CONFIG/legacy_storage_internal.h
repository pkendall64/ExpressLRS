#pragma once

#include "config.h"

static constexpr size_t LEGACY_STORAGE_SIZE = 1024U;

#if defined(PLATFORM_ESP8266)
extern "C" uint32_t _EEPROM_start;

static inline bool ReadLegacyStorageBytes(uint32_t offset, void *dst, size_t len)
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
static inline bool ReadLegacyStorageBytes(uint32_t offset, void *dst, size_t len)
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
