#include <stdint.h>
#include <string.h>
#include <vector>

#include <unity.h>

#include "flash_nvs.h"

namespace {

struct FakeFlash
{
    std::vector<uint8_t> bytes;
    size_t eraseCount = 0;
    size_t writeCount = 0;

    explicit FakeFlash(size_t totalSize)
        : bytes(totalSize, 0xFF)
    {
    }

    static bool Read(void *context, uint32_t address, void *dst, size_t len)
    {
        FakeFlash *flash = static_cast<FakeFlash *>(context);
        if (address + len > flash->bytes.size())
            return false;
        memcpy(dst, flash->bytes.data() + address, len);
        return true;
    }

    static bool Write(void *context, uint32_t address, const void *src, size_t len)
    {
        FakeFlash *flash = static_cast<FakeFlash *>(context);
        if (address + len > flash->bytes.size())
            return false;

        const uint8_t *input = static_cast<const uint8_t *>(src);
        for (size_t i = 0; i < len; ++i)
        {
            uint8_t oldValue = flash->bytes[address + i];
            uint8_t newValue = input[i];
            if ((oldValue & newValue) != newValue)
                return false;
        }
        for (size_t i = 0; i < len; ++i)
        {
            flash->bytes[address + i] &= input[i];
        }
        ++flash->writeCount;
        return true;
    }

    static bool Erase(void *context, uint32_t address, size_t len)
    {
        FakeFlash *flash = static_cast<FakeFlash *>(context);
        if (address + len > flash->bytes.size())
            return false;
        memset(flash->bytes.data() + address, 0xFF, len);
        ++flash->eraseCount;
        return true;
    }
};

flash_nvs_config_t MakeConfig(FakeFlash &flash, size_t sectorSize)
{
    flash_nvs_config_t config{};
    config.flash.context = &flash;
    config.flash.read = &FakeFlash::Read;
    config.flash.write = &FakeFlash::Write;
    config.flash.erase = &FakeFlash::Erase;
    config.sector0Address = 0;
    config.sector1Address = sectorSize;
    config.sectorSize = sectorSize;
    return config;
}

void test_nvs_commit_and_reopen_u32(void)
{
    FakeFlash flash(8192);
    FlashNVS nvs;

    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Begin(MakeConfig(flash, 4096)));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.SetU32(0x10, 0x12345678UL));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Commit());

    FlashNVS reopened;
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, reopened.Begin(MakeConfig(flash, 4096)));

    uint32_t value = 0;
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, reopened.GetU32(0x10, &value));
    TEST_ASSERT_EQUAL_HEX32(0x12345678UL, value);
}

void test_nvs_update_marks_old_record_inactive(void)
{
    FakeFlash flash(8192);
    FlashNVS nvs;

    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Begin(MakeConfig(flash, 4096)));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.SetU8(0x21, 1));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Commit());
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.SetU8(0x21, 7));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Commit());

    size_t inactiveCount = 0;
    for (size_t offset = sizeof(uint32_t) * 4; offset < 4096; offset += 4)
    {
        uint32_t key = 0;
        memcpy(&key, flash.bytes.data() + offset, sizeof(key));
        if (key == FLASH_NVS_INACTIVE_KEY)
            ++inactiveCount;
    }
    TEST_ASSERT_GREATER_THAN(0U, inactiveCount);

    uint8_t value = 0;
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.GetU8(0x21, &value));
    TEST_ASSERT_EQUAL_UINT8(7, value);
}

void test_nvs_blob_round_trip(void)
{
    FakeFlash flash(8192);
    FlashNVS nvs;
    const uint8_t payload[] = {1, 2, 3, 4, 5, 6};
    uint8_t out[8] = {};
    size_t outLen = sizeof(out);

    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Begin(MakeConfig(flash, 4096)));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.SetBlob(0x30, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Commit());
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.GetBlob(0x30, out, &outLen));
    TEST_ASSERT_EQUAL(sizeof(payload), outLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, out, sizeof(payload));
}

void test_nvs_compacts_when_sector_fills(void)
{
    FakeFlash flash(512);
    FlashNVS nvs;

    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Begin(MakeConfig(flash, 256)));
    size_t baseEraseCount = flash.eraseCount;

    for (uint32_t i = 0; i < 8; ++i)
    {
        TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.SetU32(0x40, i + 1));
        TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Commit());
    }

    TEST_ASSERT_GREATER_THAN(baseEraseCount, flash.eraseCount);

    FlashNVS reopened;
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, reopened.Begin(MakeConfig(flash, 256)));
    uint32_t value = 0;
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, reopened.GetU32(0x40, &value));
    TEST_ASSERT_EQUAL_UINT32(8, value);
}

void test_nvs_missing_commit_is_ignored_after_reopen(void)
{
    FakeFlash flash(8192);
    FlashNVS nvs;

    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Begin(MakeConfig(flash, 4096)));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.SetU8(0x50, 9));
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, nvs.Commit());

    const size_t sectorHeaderSize = sizeof(uint32_t) * 4;
    const size_t recordHeaderSize = sizeof(uint32_t) * 3 + sizeof(uint16_t) + sizeof(uint8_t) * 2;
    const size_t recordSize = ((recordHeaderSize + 1U) + 3U) & ~size_t(3U);
    const size_t commitOffset = sectorHeaderSize + recordSize;
    uint32_t emptyKey = FLASH_NVS_EMPTY_KEY;
    memcpy(flash.bytes.data() + commitOffset, &emptyKey, sizeof(emptyKey));

    FlashNVS reopened;
    TEST_ASSERT_EQUAL(FLASH_NVS_OK, reopened.Begin(MakeConfig(flash, 4096)));
    uint8_t value = 0;
    TEST_ASSERT_EQUAL(FLASH_NVS_ERR_NOT_FOUND, reopened.GetU8(0x50, &value));
}

} // namespace

void setUp() {}
void tearDown() {}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_nvs_commit_and_reopen_u32);
    RUN_TEST(test_nvs_update_marks_old_record_inactive);
    RUN_TEST(test_nvs_blob_round_trip);
    RUN_TEST(test_nvs_compacts_when_sector_fills);
    RUN_TEST(test_nvs_missing_commit_is_ignored_after_reopen);
    return UNITY_END();
}
