#pragma once
#include "SerialIO.h"

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "options.h"

class SerialCRSF final : public SerialIO, public CRSFConnector {
public:
    explicit SerialCRSF(HardwareSerial &stream, const int8_t rxPin, const int8_t txPin, const bool invert) :
        SerialIO(&stream, firmwareOptions.uart_baud, SERIAL_8N1, rxPin, txPin, invert)
    {
        crsfRouter.addConnector(this);
    }
    ~SerialCRSF() override
    {
        crsfRouter.removeConnector(this);
    }

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void forwardMessage(const crsf_header_t *message) override;

    bool sendImmediateRC() override { return true; }

private:
    CRSFParser crsfParser;

    void processBytes(uint8_t *bytes, uint16_t size) override;
};
