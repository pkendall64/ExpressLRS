#pragma once

#include "FIFO.h"
#include "SerialIO.h"
#include "options.h"
#include "telemetry_protocol.h"

// Variables / constants for Airport //
extern FIFO<AP_MAX_BUF_LEN> apInputBuffer;
extern FIFO<AP_MAX_BUF_LEN> apOutputBuffer;

class SerialAirPort final : public SerialIO
{
public:
    SerialAirPort(HardwareSerial &stream, const int8_t rxPin, const int8_t txPin)
        : SerialIO(&stream, firmwareOptions.uart_baud, SERIAL_8N1, rxPin, txPin, false) {}
    ~SerialAirPort() override = default;

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t *data) override {}

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;
};
