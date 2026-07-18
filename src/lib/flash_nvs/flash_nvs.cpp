#include "flash_nvs.h"

#include <cstring>

namespace
{
constexpr uint32_t FNV_OFFSET_BASIS = 2166136261UL;
constexpr uint32_t FNV_PRIME = 16777619UL;
constexpr size_t COPY_CHUNK_SIZE = 32U;

uint32_t crcInit()
{
    return FNV_OFFSET_BASIS;
}

uint32_t crcUpdate(uint32_t crc, const void *data, size_t len)
{
    const auto *bytes = static_cast<const uint8_t *>(data);
    while (len--)
    {
        crc ^= *bytes++;
        crc *= FNV_PRIME;
    }
    return crc;
}

bool isValidType(uint8_t type)
{
    return type == FLASH_NVS_TYPE_U8
        || type == FLASH_NVS_TYPE_U32
        || type == FLASH_NVS_TYPE_BLOB
        || type == FLASH_NVS_TYPE_COMMIT;
}
}

FlashNVS::FlashNVS()
    : m_ready(false)
    , m_activeSector(0)
    , m_spareSector(0)
    , m_generation(0)
    , m_nextTxnId(1)
    , m_writeOffset(0)
    , m_pendingBufferUsed(0)
    , m_pendingCount(0)
{
}

size_t FlashNVS::Align4(const size_t len)
{
    return (len + 3U) & ~3U;
}

uint32_t FlashNVS::CalcWordsCrc(const void *data, const size_t len)
{
    return crcUpdate(crcInit(), data, len);
}

uint32_t FlashNVS::CalcRecordCrc(const uint32_t txnId, const uint8_t type, const uint16_t length, const void *payload)
{
    uint32_t crc = crcInit();
    crc = crcUpdate(crc, &txnId, sizeof(txnId));
    crc = crcUpdate(crc, &type, sizeof(type));
    crc = crcUpdate(crc, &length, sizeof(length));
    if (length != 0 && payload != nullptr)
    {
        crc = crcUpdate(crc, payload, length);
    }
    return crc;
}

uint32_t FlashNVS::CalcHeaderCrc(const uint32_t magic, const uint32_t version, const uint32_t generation)
{
    const uint32_t fields[3] = {magic, version, generation};
    return CalcWordsCrc(fields, sizeof(fields));
}

int FlashNVS::Begin(const flash_nvs_config_t &config)
{
    if (!config.flash.read || !config.flash.write || !config.flash.erase)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (config.sectorSize < sizeof(SectorHeader) + sizeof(RecordHeader) + Align4(sizeof(uint32_t)))
        return FLASH_NVS_ERR_INVALID_ARG;
    if ((config.sectorSize & 0x3U) != 0U || config.sectorSize > FLASH_NVS_MAX_SECTOR_SIZE)
        return FLASH_NVS_ERR_INVALID_ARG;

    m_config = config;
    m_ready = false;
    m_generation = 0;
    m_nextTxnId = 1;
    m_writeOffset = sizeof(SectorHeader);
    memset(m_index, 0, sizeof(m_index));
    ResetPending();

    SectorHeader header0{};
    SectorHeader header1{};
    const bool valid0 = config.flash.read(config.flash.context, config.sector0Address, &header0, sizeof(header0))
        && header0.magic == FLASH_NVS_SECTOR_MAGIC
        && header0.version == FLASH_NVS_VERSION
        && header0.crc == CalcHeaderCrc(header0.magic, header0.version, header0.generation);
    const bool valid1 = config.flash.read(config.flash.context, config.sector1Address, &header1, sizeof(header1))
        && header1.magic == FLASH_NVS_SECTOR_MAGIC
        && header1.version == FLASH_NVS_VERSION
        && header1.crc == CalcHeaderCrc(header1.magic, header1.version, header1.generation);

    if (!valid0 && !valid1)
    {
        return FormatBlank();
    }

    int result = SelectActiveSector(header0.generation, valid0, header1.generation, valid1);
    if (result != FLASH_NVS_OK)
        return result;

    result = ScanActiveSector();
    if (result != FLASH_NVS_OK)
        return result;

    m_ready = true;
    return FLASH_NVS_OK;
}

int FlashNVS::EraseAll()
{
    if (!m_config.flash.erase)
        return FLASH_NVS_ERR_INVALID_ARG;
    return FormatBlank();
}

int FlashNVS::FormatBlank()
{
    if (!m_config.flash.erase(m_config.flash.context, m_config.sector0Address, m_config.sectorSize))
        return FLASH_NVS_ERR_FLASH;
    if (!m_config.flash.erase(m_config.flash.context, m_config.sector1Address, m_config.sectorSize))
        return FLASH_NVS_ERR_FLASH;
    if (WriteHeader(m_config.sector0Address, 1U) != FLASH_NVS_OK)
        return FLASH_NVS_ERR_FLASH;

    memset(m_index, 0, sizeof(m_index));
    ResetPending();
    m_activeSector = m_config.sector0Address;
    m_spareSector = m_config.sector1Address;
    m_generation = 1U;
    m_nextTxnId = 1U;
    m_writeOffset = sizeof(SectorHeader);
    m_ready = true;
    return FLASH_NVS_OK;
}

int FlashNVS::SelectActiveSector(const uint32_t header0Generation, const bool header0Valid, const uint32_t header1Generation, const bool header1Valid)
{
    if (header0Valid && (!header1Valid || header0Generation >= header1Generation))
    {
        m_activeSector = m_config.sector0Address;
        m_spareSector = m_config.sector1Address;
        m_generation = header0Generation;
        return FLASH_NVS_OK;
    }
    if (header1Valid)
    {
        m_activeSector = m_config.sector1Address;
        m_spareSector = m_config.sector0Address;
        m_generation = header1Generation;
        return FLASH_NVS_OK;
    }
    return FLASH_NVS_ERR_CORRUPT;
}

int FlashNVS::WriteHeader(const uint32_t sectorAddress, const uint32_t generation) const
{
    SectorHeader header{};
    header.magic = FLASH_NVS_SECTOR_MAGIC;
    header.version = FLASH_NVS_VERSION;
    header.generation = generation;
    header.crc = CalcHeaderCrc(header.magic, header.version, header.generation);
    if (!m_config.flash.write(m_config.flash.context, sectorAddress, &header, sizeof(header)))
        return FLASH_NVS_ERR_FLASH;
    return FLASH_NVS_OK;
}

int FlashNVS::ReadRecordHeader(uint32_t address, RecordHeader *header) const
{
    if (!header)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (!m_config.flash.read(m_config.flash.context, address, header, sizeof(*header)))
        return FLASH_NVS_ERR_FLASH;
    return FLASH_NVS_OK;
}

int FlashNVS::ReadPayload(const uint32_t address, void *dst, const size_t len) const
{
    if (len == 0)
        return FLASH_NVS_OK;
    if (!dst)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (!m_config.flash.read(m_config.flash.context, address + sizeof(RecordHeader), dst, len))
        return FLASH_NVS_ERR_FLASH;
    return FLASH_NVS_OK;
}

int FlashNVS::FindIndex(const uint32_t keyId) const
{
    for (size_t i = 0; i < FLASH_NVS_MAX_KEYS; ++i)
    {
        if (m_index[i].inUse && m_index[i].keyId == keyId)
            return static_cast<int>(i);
    }
    return -1;
}

int FlashNVS::FindOrCreateIndex(const uint32_t keyId)
{
    const auto existing = FindIndex(keyId);
    if (existing >= 0)
        return existing;
    for (size_t i = 0; i < FLASH_NVS_MAX_KEYS; ++i)
    {
        if (!m_index[i].inUse)
        {
            m_index[i].inUse = true;
            m_index[i].keyId = keyId;
            return static_cast<int>(i);
        }
    }
    return -1;
}

int FlashNVS::FindPending(const uint32_t keyId) const
{
    for (size_t i = 0; i < FLASH_NVS_MAX_PENDING; ++i)
    {
        if (m_pending[i].inUse && m_pending[i].keyId == keyId)
            return static_cast<int>(i);
    }
    return -1;
}

int FlashNVS::ResetPending()
{
    memset(m_pending, 0, sizeof(m_pending));
    memset(m_pendingBuffer, 0xFF, sizeof(m_pendingBuffer));
    m_pendingBufferUsed = 0;
    m_pendingCount = 0;
    return FLASH_NVS_OK;
}

int FlashNVS::ApplyLiveEntry(const uint32_t address, const RecordHeader &header)
{
    const auto index = FindOrCreateIndex(header.keyId);
    if (index < 0)
        return FLASH_NVS_ERR_NO_SPACE;
    m_index[index].keyId = header.keyId;
    m_index[index].address = address;
    m_index[index].length = header.length;
    m_index[index].type = header.type;
    m_index[index].inUse = true;
    return FLASH_NVS_OK;
}

int FlashNVS::ScanSector(const uint32_t sectorAddress, uint32_t *writeOffset, const bool applyIndex)
{
    RecordHeader stagedHeaders[FLASH_NVS_MAX_PENDING];
    uint32_t stagedAddresses[FLASH_NVS_MAX_PENDING];
    size_t stagedCount = 0;
    size_t stagedActiveCount = 0;
    uint32_t offset = sizeof(SectorHeader);
    uint8_t buffer[COPY_CHUNK_SIZE];

    while (offset + sizeof(RecordHeader) <= m_config.sectorSize)
    {
        RecordHeader header{};
        int result = ReadRecordHeader(sectorAddress + offset, &header);
        if (result != FLASH_NVS_OK)
            return result;
        if (header.keyId == FLASH_NVS_EMPTY_KEY)
        {
            *writeOffset = offset;
            return FLASH_NVS_OK;
        }
        if (!isValidType(header.type))
            return FLASH_NVS_ERR_CORRUPT;

        const auto totalSize = Align4(sizeof(RecordHeader) + header.length);
        if (offset + totalSize > m_config.sectorSize)
            return FLASH_NVS_ERR_CORRUPT;

        uint32_t crc = crcInit();
        crc = crcUpdate(crc, &header.txnId, sizeof(header.txnId));
        crc = crcUpdate(crc, &header.type, sizeof(header.type));
        crc = crcUpdate(crc, &header.length, sizeof(header.length));

        size_t remaining = header.length;
        uint32_t payloadAddress = sectorAddress + offset + sizeof(RecordHeader);
        while (remaining > 0)
        {
            const size_t chunk = remaining > sizeof(buffer) ? sizeof(buffer) : remaining;
            if (!m_config.flash.read(m_config.flash.context, payloadAddress, buffer, chunk))
                return FLASH_NVS_ERR_FLASH;
            crc = crcUpdate(crc, buffer, chunk);
            payloadAddress += chunk;
            remaining -= chunk;
        }
        if (crc != header.crc)
            return FLASH_NVS_ERR_CORRUPT;

        if (header.keyId == FLASH_NVS_INACTIVE_KEY)
        {
            if (stagedCount >= FLASH_NVS_MAX_PENDING)
                return FLASH_NVS_ERR_NO_SPACE;
            ++stagedCount;
            offset += totalSize;
            continue;
        }

        if (header.keyId == FLASH_NVS_COMMIT_KEY)
        {
            if (header.type != FLASH_NVS_TYPE_COMMIT || header.length != sizeof(uint32_t))
                return FLASH_NVS_ERR_CORRUPT;

            uint32_t entryCount = 0;
            result = ReadPayload(sectorAddress + offset, &entryCount, sizeof(entryCount));
            if (result != FLASH_NVS_OK)
                return result;
            if (entryCount != stagedCount)
                return FLASH_NVS_ERR_CORRUPT;
            if (header.txnId >= m_nextTxnId)
                m_nextTxnId = header.txnId + 1U;
            if (applyIndex)
            {
                for (size_t i = 0; i < stagedActiveCount; ++i)
                {
                    result = ApplyLiveEntry(stagedAddresses[i], stagedHeaders[i]);
                    if (result != FLASH_NVS_OK)
                        return result;
                }
            }
            stagedCount = 0;
            stagedActiveCount = 0;
            offset += totalSize;
            continue;
        }

        if (stagedActiveCount >= FLASH_NVS_MAX_PENDING || stagedCount >= FLASH_NVS_MAX_PENDING)
            return FLASH_NVS_ERR_NO_SPACE;
        stagedHeaders[stagedActiveCount] = header;
        stagedAddresses[stagedActiveCount] = sectorAddress + offset;
        ++stagedActiveCount;
        ++stagedCount;
        offset += totalSize;
    }

    *writeOffset = offset;
    return FLASH_NVS_OK;
}

int FlashNVS::ScanActiveSector()
{
    memset(m_index, 0, sizeof(m_index));
    m_nextTxnId = 1U;
    return ScanSector(m_activeSector, &m_writeOffset, true);
}

int FlashNVS::GetU8(const uint32_t keyId, uint8_t *value) const
{
    if (!m_ready || !value)
        return FLASH_NVS_ERR_INVALID_ARG;
    const auto index = FindIndex(keyId);
    if (index < 0)
        return FLASH_NVS_ERR_NOT_FOUND;
    if (m_index[index].type != FLASH_NVS_TYPE_U8 || m_index[index].length != sizeof(uint8_t))
        return FLASH_NVS_ERR_CORRUPT;
    return ReadPayload(m_index[index].address, value, sizeof(uint8_t));
}

int FlashNVS::GetU32(const uint32_t keyId, uint32_t *value) const
{
    if (!m_ready || !value)
        return FLASH_NVS_ERR_INVALID_ARG;
    const auto index = FindIndex(keyId);
    if (index < 0)
        return FLASH_NVS_ERR_NOT_FOUND;
    if (m_index[index].type != FLASH_NVS_TYPE_U32 || m_index[index].length != sizeof(uint32_t))
        return FLASH_NVS_ERR_CORRUPT;
    return ReadPayload(m_index[index].address, value, sizeof(uint32_t));
}

int FlashNVS::GetBlob(const uint32_t keyId, void *value, size_t *len) const
{
    if (!m_ready || !len)
        return FLASH_NVS_ERR_INVALID_ARG;
    const auto index = FindIndex(keyId);
    if (index < 0)
        return FLASH_NVS_ERR_NOT_FOUND;
    if (m_index[index].type != FLASH_NVS_TYPE_BLOB)
        return FLASH_NVS_ERR_CORRUPT;
    if (!value)
    {
        *len = m_index[index].length;
        return FLASH_NVS_OK;
    }
    if (*len < m_index[index].length)
    {
        *len = m_index[index].length;
        return FLASH_NVS_ERR_NO_SPACE;
    }
    *len = m_index[index].length;
    return ReadPayload(m_index[index].address, value, m_index[index].length);
}

int FlashNVS::SetU8(const uint32_t keyId, const uint8_t value)
{
    return StageValue(keyId, FLASH_NVS_TYPE_U8, &value, sizeof(value));
}

int FlashNVS::SetU32(const uint32_t keyId, const uint32_t value)
{
    return StageValue(keyId, FLASH_NVS_TYPE_U32, &value, sizeof(value));
}

int FlashNVS::SetBlob(const uint32_t keyId, const void *value, const size_t len)
{
    return StageValue(keyId, FLASH_NVS_TYPE_BLOB, value, len);
}

int FlashNVS::StageValue(const uint32_t keyId, const uint8_t type, const void *payload, const size_t len)
{
    if (!m_ready)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (keyId == FLASH_NVS_EMPTY_KEY || keyId == FLASH_NVS_INACTIVE_KEY || keyId == FLASH_NVS_COMMIT_KEY)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (!payload && len != 0)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (!isValidType(type) || type == FLASH_NVS_TYPE_COMMIT)
        return FLASH_NVS_ERR_INVALID_ARG;

    const auto totalSize = Align4(sizeof(RecordHeader) + len);
    if (totalSize > m_config.sectorSize)
        return FLASH_NVS_ERR_DATA_TOO_LONG;
    if (m_pendingBufferUsed + totalSize > sizeof(m_pendingBuffer))
        return FLASH_NVS_ERR_NO_SPACE;

    const auto existingPending = FindPending(keyId);
    if (existingPending >= 0)
    {
        m_pending[existingPending].inUse = false;
        if (m_pendingCount > 0)
            --m_pendingCount;
    }

    int freePending = -1;
    for (size_t i = 0; i < FLASH_NVS_MAX_PENDING; ++i)
    {
        if (!m_pending[i].inUse)
        {
            freePending = static_cast<int>(i);
            break;
        }
    }
    if (freePending < 0)
        return FLASH_NVS_ERR_NO_SPACE;

    memset(m_pendingBuffer + m_pendingBufferUsed, 0xFF, totalSize);
    auto *header = reinterpret_cast<RecordHeader *>(m_pendingBuffer + m_pendingBufferUsed);
    header->keyId = keyId;
    header->txnId = 0;
    header->length = static_cast<uint16_t>(len);
    header->type = type;
    header->reserved = 0xFF;
    header->crc = CalcRecordCrc(0U, type, header->length, payload);
    if (len != 0)
        memcpy(m_pendingBuffer + m_pendingBufferUsed + sizeof(RecordHeader), payload, len);

    const auto currentIndex = FindIndex(keyId);
    m_pending[freePending].keyId = keyId;
    m_pending[freePending].priorAddress = currentIndex >= 0 ? m_index[currentIndex].address : 0U;
    m_pending[freePending].bufferOffset = m_pendingBufferUsed;
    m_pending[freePending].totalSize = totalSize;
    m_pending[freePending].inUse = true;

    m_pendingBufferUsed += totalSize;
    ++m_pendingCount;
    return FLASH_NVS_OK;
}

int FlashNVS::AppendPendingEntry(const PendingEntry &pending, const uint32_t txnId, uint32_t *newAddress)
{
    auto *header = reinterpret_cast<RecordHeader *>(m_pendingBuffer + pending.bufferOffset);
    const uint8_t *payload = m_pendingBuffer + pending.bufferOffset + sizeof(RecordHeader);
    header->txnId = txnId;
    header->crc = CalcRecordCrc(txnId, header->type, header->length, payload);

    const uint32_t address = m_activeSector + m_writeOffset;
    if (!m_config.flash.write(m_config.flash.context, address + sizeof(uint32_t), reinterpret_cast<uint8_t *>(header) + sizeof(uint32_t), pending.totalSize - sizeof(uint32_t)))
        return FLASH_NVS_ERR_FLASH;
    if (!m_config.flash.write(m_config.flash.context, address, &header->keyId, sizeof(header->keyId)))
        return FLASH_NVS_ERR_FLASH;

    m_writeOffset += static_cast<uint32_t>(pending.totalSize);
    if (newAddress)
        *newAddress = address;
    return FLASH_NVS_OK;
}

int FlashNVS::AppendCommitRecord(const uint32_t txnId, const uint32_t entryCount, uint32_t *newAddress)
{
    enum { commitRecordSize = sizeof(RecordHeader) + sizeof(uint32_t) };
    uint8_t buffer[commitRecordSize];

    auto *header = reinterpret_cast<RecordHeader *>(buffer);
    header->keyId = FLASH_NVS_COMMIT_KEY;
    header->txnId = txnId;
    header->length = sizeof(uint32_t);
    header->type = FLASH_NVS_TYPE_COMMIT;
    header->reserved = 0xFF;
    memcpy(buffer + sizeof(RecordHeader), &entryCount, sizeof(entryCount));
    header->crc = CalcRecordCrc(txnId, header->type, header->length, buffer + sizeof(RecordHeader));

    uint32_t address = m_activeSector + m_writeOffset;
    if (!m_config.flash.write(m_config.flash.context, address + sizeof(uint32_t), buffer + sizeof(uint32_t), sizeof(buffer) - sizeof(uint32_t)))
        return FLASH_NVS_ERR_FLASH;
    if (!m_config.flash.write(m_config.flash.context, address, &header->keyId, sizeof(header->keyId)))
        return FLASH_NVS_ERR_FLASH;

    m_writeOffset += sizeof(buffer);
    if (newAddress)
        *newAddress = address;
    return FLASH_NVS_OK;
}

int FlashNVS::InvalidateRecord(const uint32_t address) const
{
    if (address == 0U)
        return FLASH_NVS_OK;
    constexpr uint32_t inactive = FLASH_NVS_INACTIVE_KEY;
    if (!m_config.flash.write(m_config.flash.context, address, &inactive, sizeof(inactive)))
        return FLASH_NVS_ERR_FLASH;
    return FLASH_NVS_OK;
}

int FlashNVS::EnsureSpace(const size_t bytesNeeded)
{
    if (m_writeOffset + bytesNeeded <= m_config.sectorSize)
        return FLASH_NVS_OK;
    const auto result = Compact();
    if (result != FLASH_NVS_OK)
        return result;
    return (m_writeOffset + bytesNeeded <= m_config.sectorSize) ? FLASH_NVS_OK : FLASH_NVS_ERR_NO_SPACE;
}

int FlashNVS::Commit()
{
    if (!m_ready)
        return FLASH_NVS_ERR_INVALID_ARG;
    if (m_pendingCount == 0)
        return FLASH_NVS_OK;

    size_t bytesNeeded = Align4(sizeof(RecordHeader) + sizeof(uint32_t));
    uint32_t validCount = 0;
    for (const auto & i : m_pending)
    {
        if (!i.inUse)
            continue;
        bytesNeeded += i.totalSize;
        ++validCount;
    }

    int result = EnsureSpace(bytesNeeded);
    if (result != FLASH_NVS_OK)
        return result;

    const uint32_t txnId = m_nextTxnId++;
    struct AppliedEntry { size_t pendingIndex; uint32_t address; } applied[FLASH_NVS_MAX_PENDING];
    size_t appliedCount = 0;

    for (size_t i = 0; i < FLASH_NVS_MAX_PENDING; ++i)
    {
        if (!m_pending[i].inUse)
            continue;
        uint32_t newAddress = 0;
        result = AppendPendingEntry(m_pending[i], txnId, &newAddress);
        if (result != FLASH_NVS_OK)
            return result;
        applied[appliedCount].pendingIndex = i;
        applied[appliedCount].address = newAddress;
        ++appliedCount;
    }

    result = AppendCommitRecord(txnId, validCount, nullptr);
    if (result != FLASH_NVS_OK)
        return result;

    for (size_t i = 0; i < appliedCount; ++i)
    {
        PendingEntry &pending = m_pending[applied[i].pendingIndex];
        result = InvalidateRecord(pending.priorAddress);
        if (result != FLASH_NVS_OK)
            return result;

        RecordHeader header = *reinterpret_cast<RecordHeader *>(m_pendingBuffer + pending.bufferOffset);
        header.txnId = txnId;
        header.crc = CalcRecordCrc(txnId, header.type, header.length, m_pendingBuffer + pending.bufferOffset + sizeof(RecordHeader));
        result = ApplyLiveEntry(applied[i].address, header);
        if (result != FLASH_NVS_OK)
            return result;
    }

    ResetPending();
    return FLASH_NVS_OK;
}

int FlashNVS::Compact()
{
    if (!m_ready)
        return FLASH_NVS_ERR_INVALID_ARG;

    if (!m_config.flash.erase(m_config.flash.context, m_spareSector, m_config.sectorSize))
        return FLASH_NVS_ERR_FLASH;

    const uint32_t oldActive = m_activeSector;
    const uint32_t oldGeneration = m_generation;
    const uint32_t newTxnId = m_nextTxnId++;
    uint32_t newWriteOffset = sizeof(SectorHeader);
    uint8_t copyBuffer[COPY_CHUNK_SIZE];
    uint32_t liveCount = 0;

    for (auto & i : m_index)
    {
        if (!i.inUse)
            continue;

        RecordHeader header{};
        const auto result = ReadRecordHeader(i.address, &header);
        if (result != FLASH_NVS_OK)
            return result;

        size_t totalSize = Align4(sizeof(RecordHeader) + header.length);
        if (newWriteOffset + totalSize + Align4(sizeof(RecordHeader) + sizeof(uint32_t)) > m_config.sectorSize)
            return FLASH_NVS_ERR_NO_SPACE;

        RecordHeader newHeader{};
        newHeader.keyId = header.keyId;
        newHeader.txnId = newTxnId;
        newHeader.length = header.length;
        newHeader.type = header.type;
        newHeader.reserved = 0xFF;

        uint32_t crc = crcInit();
        crc = crcUpdate(crc, &newHeader.txnId, sizeof(newHeader.txnId));
        crc = crcUpdate(crc, &newHeader.type, sizeof(newHeader.type));
        crc = crcUpdate(crc, &newHeader.length, sizeof(newHeader.length));

        const uint32_t dstAddress = m_spareSector + newWriteOffset;
        if (!m_config.flash.write(m_config.flash.context, dstAddress + sizeof(uint32_t), reinterpret_cast<uint8_t *>(&newHeader) + sizeof(uint32_t), offsetof(RecordHeader, crc) - sizeof(uint32_t)))
            return FLASH_NVS_ERR_FLASH;

        size_t remaining = header.length;
        uint32_t srcPayloadAddress = i.address + sizeof(RecordHeader);
        uint32_t dstPayloadAddress = dstAddress + sizeof(RecordHeader);
        while (remaining > 0)
        {
            const size_t chunk = remaining > sizeof(copyBuffer) ? sizeof(copyBuffer) : remaining;
            if (!m_config.flash.read(m_config.flash.context, srcPayloadAddress, copyBuffer, chunk))
                return FLASH_NVS_ERR_FLASH;
            crc = crcUpdate(crc, copyBuffer, chunk);
            const size_t writeSize = Align4(chunk);
            if (writeSize != chunk)
                memset(copyBuffer + chunk, 0xFF, writeSize - chunk);
            if (!m_config.flash.write(m_config.flash.context, dstPayloadAddress, copyBuffer, writeSize))
                return FLASH_NVS_ERR_FLASH;
            srcPayloadAddress += chunk;
            dstPayloadAddress += static_cast<uint32_t>(writeSize);
            remaining -= chunk;
        }

        newHeader.crc = crc;
        if (!m_config.flash.write(m_config.flash.context, dstAddress + offsetof(RecordHeader, crc), &newHeader.crc, sizeof(newHeader.crc)))
            return FLASH_NVS_ERR_FLASH;
        if (!m_config.flash.write(m_config.flash.context, dstAddress, &newHeader.keyId, sizeof(newHeader.keyId)))
            return FLASH_NVS_ERR_FLASH;

        i.address = dstAddress;
        newWriteOffset += static_cast<uint32_t>(totalSize);
        ++liveCount;
    }

    enum { commitRecordSize = sizeof(RecordHeader) + sizeof(uint32_t) };
    uint8_t commitBuffer[commitRecordSize];
    auto *commitHeader = reinterpret_cast<RecordHeader *>(commitBuffer);
    commitHeader->keyId = FLASH_NVS_COMMIT_KEY;
    commitHeader->txnId = newTxnId;
    commitHeader->length = sizeof(uint32_t);
    commitHeader->type = FLASH_NVS_TYPE_COMMIT;
    commitHeader->reserved = 0xFF;
    memcpy(commitBuffer + sizeof(RecordHeader), &liveCount, sizeof(liveCount));
    commitHeader->crc = CalcRecordCrc(newTxnId, commitHeader->type, commitHeader->length, commitBuffer + sizeof(RecordHeader));

    const uint32_t commitAddress = m_spareSector + newWriteOffset;
    if (!m_config.flash.write(m_config.flash.context, commitAddress + sizeof(uint32_t), commitBuffer + sizeof(uint32_t), sizeof(commitBuffer) - sizeof(uint32_t)))
        return FLASH_NVS_ERR_FLASH;
    if (!m_config.flash.write(m_config.flash.context, commitAddress, &commitHeader->keyId, sizeof(commitHeader->keyId)))
        return FLASH_NVS_ERR_FLASH;
    newWriteOffset += sizeof(commitBuffer);

    const auto result = WriteHeader(m_spareSector, oldGeneration + 1U);
    if (result != FLASH_NVS_OK)
        return result;

    for (auto & i : m_pending)
    {
        if (!i.inUse)
            continue;
        const int index = FindIndex(i.keyId);
        i.priorAddress = index >= 0 ? m_index[index].address : 0U;
    }
    if (!m_config.flash.erase(m_config.flash.context, oldActive, m_config.sectorSize))
        return FLASH_NVS_ERR_FLASH;

    m_activeSector = m_spareSector;
    m_spareSector = oldActive;
    m_generation = oldGeneration + 1U;
    m_writeOffset = newWriteOffset;
    return FLASH_NVS_OK;
}
