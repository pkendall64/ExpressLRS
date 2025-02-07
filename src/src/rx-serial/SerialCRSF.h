#pragma once
#include "SerialIO.h"
#include "options.h"

#include "CRSFRouter.h"

class SerialCRSF final : public SerialIO, public CRSFConnector
{
public:
    SerialCRSF(HardwareSerial &stream, const int8_t rxPin, const int8_t txPin, const bool invert)
        : SerialIO(&stream, firmwareOptions.uart_baud, SERIAL_8N1, rxPin, txPin, invert)
    {
        crsfRouter.addConnector(this);
    }
    ~SerialCRSF() override
    {
        crsfRouter.removeConnector(this);
    }

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void forwardMessage(const crsf_header_t *message) override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    bool sendImmediateRC() override { return true; }

private:
    void processBytes(uint8_t *bytes, uint16_t size) override;
};
