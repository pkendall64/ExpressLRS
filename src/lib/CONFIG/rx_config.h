#pragma once

#include "targets.h"
#include "common.h"

#if defined(PLATFORM_ESP32)
#include <nvs_flash.h>
#include <nvs.h>
#endif

#define RX_CONFIG_VERSION   11U

constexpr uint8_t PWM_MAX_CHANNELS = 16;

typedef enum : uint8_t {
    BINDSTORAGE_PERSISTENT = 0,
    BINDSTORAGE_VOLATILE = 1,
    BINDSTORAGE_RETURNABLE = 2,
    BINDSTORAGE_ADMINISTERED = 3,
} rx_config_bindstorage_t;

typedef union {
    struct {
        uint32_t failsafe:11,    // us output during failsafe +476 (e.g. 1024 here would be 1500us)
                 inputChannel:4, // 0-based input channel
                 inverted:1,     // invert channel output
                 mode:4,         // Output mode (eServoOutputMode)
                 stretched:1,    // expand the channel input to 500us - 2500us
                 narrow:1,       // Narrow output mode (half pulse width)
                 failsafeMode:2, // failsafe output mode (eServoOutputFailsafeMode)
                 unused:8;       // FUTURE: When someone complains "everyone" uses inverted polarity PWM or something :/
    } val;
    uint32_t raw;
} rx_config_pwm_t;

typedef struct __attribute__((packed)) {
    uint32_t    version;
    uint8_t     uid[UID_LEN];
    uint8_t     unused_padding;
    uint8_t     serial1Protocol:4,  // secondary serial protocol
                serial1Protocol_unused:4;
    uint32_t    flash_discriminator;
    struct __attribute__((packed)) {
        uint16_t    scale;          // FUTURE: Override compiled vbat scale
        int16_t     offset;         // FUTURE: Override comiled vbat offset
    } vbat;
    uint8_t     bindStorage:2,     // rx_config_bindstorage_t
                power:4,
                antennaMode:2;      // 0=0, 1=1, 2=Diversity
    uint8_t     powerOnCounter:2,
                forceTlmOff:1,
                rateInitialIdx:5;   // Rate to start rateCycling at on boot
    uint8_t     modelId;
    uint8_t     serialProtocol:4,
                failsafeMode:2,
                relayEnabled:1,
                unused:1;
    rx_config_pwm_t pwmChannels[PWM_MAX_CHANNELS] __attribute__((aligned(4)));
    uint8_t     teamraceChannel:4,
                teamracePosition:3,
                teamracePitMode:1;  // FUTURE: Enable pit mode when disabling model
    uint8_t     targetSysId;
    uint8_t     sourceSysId;
} rx_config_t;

class RxConfig
{
public:
    RxConfig();

    void Load();
    uint32_t Commit();

    // Getters
    bool     GetIsBound() const;
    const uint8_t* GetUID() const { return m_config.uid; }
    bool GetRelayEnabled() const { return m_config.relayEnabled; }
#if defined(PLATFORM_ESP8266)
    uint8_t  GetPowerOnCounter() const;
#else
    uint8_t  GetPowerOnCounter() const { return m_config.powerOnCounter; }
#endif
    uint8_t  GetModelId() const { return m_config.modelId; }
    uint8_t GetPower() const { return m_config.power; }
    uint8_t GetAntennaMode() const { return m_config.antennaMode; }
    bool     IsModified() const { return m_modified != 0; }
    const rx_config_pwm_t *GetPwmChannel(uint8_t ch) const { return &m_config.pwmChannels[ch]; }
    bool GetForceTlmOff() const { return m_config.forceTlmOff; }
    uint8_t GetRateInitialIdx() const { return m_config.rateInitialIdx; }
    eSerialProtocol GetSerialProtocol() const { return (eSerialProtocol)m_config.serialProtocol; }
#if defined(PLATFORM_ESP32)
    eSerial1Protocol GetSerial1Protocol() const { return (eSerial1Protocol)m_config.serial1Protocol; }
#endif
    uint8_t GetTeamraceChannel() const { return m_config.teamraceChannel; }
    uint8_t GetTeamracePosition() const { return m_config.teamracePosition; }
    eFailsafeMode GetFailsafeMode() const { return (eFailsafeMode)m_config.failsafeMode; }
    uint8_t GetTargetSysId()  const { return m_config.targetSysId; }
    uint8_t GetSourceSysId()  const { return m_config.sourceSysId; }
    rx_config_bindstorage_t GetBindStorage() const { return (rx_config_bindstorage_t)m_config.bindStorage; }
    bool IsOnLoan() const;

    // Setters
    void SetUID(uint8_t* uid);
    void SetRelayEnabled(bool relayEnabled);
    void SetPowerOnCounter(uint8_t powerOnCounter);
    void SetModelId(uint8_t modelId);
    void SetPower(uint8_t power);
    void SetAntennaMode(uint8_t antennaMode);
    void SetDefaults(bool commit);
    void SetPwmChannel(uint8_t ch, uint16_t failsafe, uint8_t inputCh, bool inverted, uint8_t mode, uint8_t stretched);
    void SetPwmChannelRaw(uint8_t ch, uint32_t raw);
    void SetForceTlmOff(bool forceTlmOff);
    void SetRateInitialIdx(uint8_t rateInitialIdx);
    void SetSerialProtocol(eSerialProtocol serialProtocol);
#if defined(PLATFORM_ESP32)
    void SetSerial1Protocol(eSerial1Protocol serial1Protocol);
#endif
    void SetTeamraceChannel(uint8_t teamraceChannel);
    void SetTeamracePosition(uint8_t teamracePosition);
    void SetFailsafeMode(eFailsafeMode failsafeMode);
    void SetTargetSysId(uint8_t sysID);
    void SetSourceSysId(uint8_t sysID);
    void SetBindStorage(rx_config_bindstorage_t value);
    void ReturnLoan();

private:
    void CheckUpdateFlashedUid(bool skipDescrimCheck);
    void UpgradeUid(uint8_t *onLoanUid, uint8_t *boundUid);
    void UpgradeEepromV4();
    void UpgradeEepromV5();
    void UpgradeEepromV6();
    void UpgradeEepromV7V8(uint8_t ver);
    void UpgradeEepromV9V10(uint8_t ver);

    rx_config_t m_config;
    uint32_t    m_modified;
};

extern RxConfig config;
