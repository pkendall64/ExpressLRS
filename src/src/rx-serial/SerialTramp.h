#pragma once

#include "SerialIO.h"

#define TRAMP_FRAME_SIZE 16
#define TRAMP_HEADER 0x0F
// check value for MSP_SET_VTX_CONFIG to determine if value is encoded
// band/channel or frequency in MHz (3 bits for band and 3 bits for channel)
#define VTXCOMMON_MSP_BANDCHAN_CHKVAL ((uint16_t)((7 << 3) + 7))

class SerialTramp final : public SerialIO {
public:
    explicit SerialTramp(HardwareSerial &stream, const int8_t txPin) :
        SerialIO(&stream, 9600, SERIAL_8N1, UNDEF_PIN, txPin, false)
    {
#if defined(PLATFORM_ESP32)
        // we are on UART1, use Serial1 TX assigned pin for half duplex
        UTXDoutIdx = U1TXD_OUT_IDX;
        URXDinIdx = U1RXD_IN_IDX;
        halfDuplexPin = txPin;
#endif
        setRXMode();
    }

    ~SerialTramp() override = default;

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t* data) override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }
private:
    void processBytes(uint8_t *bytes, uint16_t size) override {}
    void setTXMode() const;
    void setRXMode() const;
#if defined(PLATFORM_ESP32)
    int8_t halfDuplexPin;
    uint8_t UTXDoutIdx;
    uint8_t URXDinIdx;
#endif
};
