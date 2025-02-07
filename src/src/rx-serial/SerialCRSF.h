#pragma once

#include "SerialIO.h"
#include "options.h"

class SerialCRSF final : public SerialIO
{
public:
    SerialCRSF(HardwareSerial &stream, const int8_t rxPin, const int8_t txPin, const bool invert)
        : SerialIO(&stream, firmwareOptions.uart_baud, SERIAL_8N1, rxPin, txPin, invert) {}
    ~SerialCRSF() override = default;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void queueMSPFrameTransmission(uint8_t *data) override;
    void queueLinkStatisticsPacket() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    bool sendImmediateRC() override { return true; }

private:
    void processBytes(uint8_t *bytes, uint16_t size) override;
};
