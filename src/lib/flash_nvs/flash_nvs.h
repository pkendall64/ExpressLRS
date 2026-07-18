#pragma once

#include <cstddef>
#include <cstdint>

#define FLASH_NVS_SECTOR_MAGIC 0x534E5645UL
#define FLASH_NVS_VERSION      1UL
#define FLASH_NVS_EMPTY_KEY    0xFFFFFFFFUL
#define FLASH_NVS_INACTIVE_KEY 0x00000000UL
#define FLASH_NVS_COMMIT_KEY   0xFFFFFFFEUL

#define FLASH_NVS_MAX_SECTOR_SIZE 4096U
#define FLASH_NVS_MAX_KEYS        128U
#define FLASH_NVS_MAX_PENDING     128U

enum flash_nvs_result_t
{
    FLASH_NVS_OK = 0,
    FLASH_NVS_ERR_INVALID_ARG = 1,
    FLASH_NVS_ERR_FLASH = 2,
    FLASH_NVS_ERR_NOT_FOUND = 3,
    FLASH_NVS_ERR_NO_SPACE = 4,
    FLASH_NVS_ERR_DATA_TOO_LONG = 5,
    FLASH_NVS_ERR_CORRUPT = 6,
};

enum flash_nvs_type_t : uint8_t
{
    FLASH_NVS_TYPE_U8 = 1,
    FLASH_NVS_TYPE_U32 = 2,
    FLASH_NVS_TYPE_BLOB = 3,
    FLASH_NVS_TYPE_COMMIT = 4,
};

struct flash_nvs_flash_driver_t
{
    void *context;
    bool (*read)(void *context, uint32_t address, void *dst, size_t len);
    bool (*write)(void *context, uint32_t address, const void *src, size_t len);
    bool (*erase)(void *context, uint32_t address, size_t len);
};

struct flash_nvs_config_t
{
    flash_nvs_flash_driver_t flash;
    uint32_t sector0Address;
    uint32_t sector1Address;
    size_t sectorSize;
};

class FlashNVS
{
public:
    FlashNVS();

    int Begin(const flash_nvs_config_t &config);
    int EraseAll();
    int Compact();

    int GetU8(uint32_t keyId, uint8_t *value) const;
    int GetU32(uint32_t keyId, uint32_t *value) const;
    int GetBlob(uint32_t keyId, void *value, size_t *len) const;

    int SetU8(uint32_t keyId, uint8_t value);
    int SetU32(uint32_t keyId, uint32_t value);
    int SetBlob(uint32_t keyId, const void *value, size_t len);
    int Commit();

private:
    struct SectorHeader
    {
        uint32_t magic;
        uint32_t version;
        uint32_t generation;
        uint32_t crc;
    };

    struct RecordHeader
    {
        uint32_t keyId;
        uint32_t txnId;
        uint16_t length;
        uint8_t type;
        uint8_t reserved;
        uint32_t crc;
    };

    struct IndexEntry
    {
        uint32_t keyId;
        uint32_t address;
        uint16_t length;
        uint8_t type;
        bool inUse;
    };

    struct PendingEntry
    {
        uint32_t keyId;
        uint32_t priorAddress;
        size_t bufferOffset;
        size_t totalSize;
        bool inUse;
    };

    static size_t Align4(size_t len);
    static uint32_t CalcWordsCrc(const void *data, size_t len);
    static uint32_t CalcRecordCrc(uint32_t txnId, uint8_t type, uint16_t length, const void *payload);
    static uint32_t CalcHeaderCrc(uint32_t magic, uint32_t version, uint32_t generation);

    int FormatBlank();
    int ScanActiveSector();
    int ScanSector(uint32_t sectorAddress, uint32_t *writeOffset, bool applyIndex);
    int ReadRecordHeader(uint32_t address, RecordHeader *header) const;
    int ReadPayload(uint32_t address, void *dst, size_t len) const;
    int WriteHeader(uint32_t sectorAddress, uint32_t generation) const;
    int StageValue(uint32_t keyId, uint8_t type, const void *payload, size_t len);
    int AppendPendingEntry(const PendingEntry &pending, uint32_t txnId, uint32_t *newAddress);
    int AppendCommitRecord(uint32_t txnId, uint32_t entryCount, uint32_t *newAddress);
    int InvalidateRecord(uint32_t address) const;
    int EnsureSpace(size_t bytesNeeded);
    int ApplyLiveEntry(uint32_t address, const RecordHeader &header);
    int FindIndex(uint32_t keyId) const;
    int FindOrCreateIndex(uint32_t keyId);
    int FindPending(uint32_t keyId) const;
    int ResetPending();
    int SelectActiveSector(uint32_t header0Generation, bool header0Valid, uint32_t header1Generation, bool header1Valid);

    flash_nvs_config_t m_config{};
    bool m_ready;
    uint32_t m_activeSector;
    uint32_t m_spareSector;
    uint32_t m_generation;
    uint32_t m_nextTxnId;
    uint32_t m_writeOffset;
    IndexEntry m_index[FLASH_NVS_MAX_KEYS]{};
    PendingEntry m_pending[FLASH_NVS_MAX_PENDING]{};
    uint8_t m_pendingBuffer[FLASH_NVS_MAX_SECTOR_SIZE]{};
    size_t m_pendingBufferUsed;
    size_t m_pendingCount;
};
