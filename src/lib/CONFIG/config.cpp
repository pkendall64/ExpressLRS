#include "config.h"
#include "common.h"
#include "device.h"
#include "POWERMGNT.h"
#include "OTA.h"
#include "helpers.h"
#include "logging.h"

#if defined(PLATFORM_ESP32)
#include <mbedtls/md5.h> // for SetBindPhrase()
#endif

void BindphraseConfigurable::SetBindPhrase(uint8_t *phrase, size_t phraseLen)
{
    constexpr uint8_t BIND_KEY[] = "-DMY_BINDING_PHRASE=\"";

    uint8_t UID_md5[16] {};
    // A blank binding phrase will just use the UID_md5 set to all zeroes, which is unbound
    if (phraseLen)
    {
#if defined(PLATFORM_ESP8266)
        md5_context_t md5;
        MD5Init(&md5);
        MD5Update(&md5, BIND_KEY, sizeof(BIND_KEY)-1);
        MD5Update(&md5, phrase, phraseLen);
        MD5Update(&md5, &BIND_KEY[sizeof(BIND_KEY)-2], 1); // just the " from the end
        MD5Final(UID_md5, &md5);
#elif defined(PLATFORM_ESP32)
        mbedtls_md5_context md5;
        mbedtls_md5_init(&md5);
        mbedtls_md5_update_ret(&md5, BIND_KEY, sizeof(BIND_KEY)-1);
        mbedtls_md5_update_ret(&md5, phrase, phraseLen);
        mbedtls_md5_update_ret(&md5, &BIND_KEY[sizeof(BIND_KEY)-2], 1);
        mbedtls_md5_finish_ret(&md5, UID_md5);
        mbedtls_md5_free(&md5);
#endif
    }

    // UID is the first UID_LEN of the md5
    SetUID(UID_md5);
}
