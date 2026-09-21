#pragma once
#include "SerialIO.h"
#include "ELRSSerial.h"

#include "CRSFParser.h"
#include "CRSFRouter.h"

class SerialCRSF final : public SerialIO, public CRSFConnector {
public:
    SerialCRSF(ELRSSerial &port, uint32_t baud, int8_t rxPin, int8_t txPin, bool invert)
        : SerialIO(&port, &port)
    {
        port.begin(baud, SERIAL_8N1, rxPin, txPin, invert);
        crsfRouter.addConnector(this);
    }
    ~SerialCRSF() override
    {
        crsfRouter.removeConnector(this);
    }

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void forwardMessage(const crsf_header_t *message) override;

    bool sendImmediateRC() override { return true; }

private:
    CRSFParser crsfParser;

    void processBytes(uint8_t *bytes, uint16_t size) override;
};
