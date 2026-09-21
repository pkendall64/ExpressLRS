#pragma once

#include "SerialIO.h"
#include "ELRSSerial.h"
#include "CRSFRouter.h"
#include "device.h"

class SerialSmartAudio final : public SerialIO, public CRSFConnector {
public:
    SerialSmartAudio(ELRSSerial &port, int8_t txPin);
    ~SerialSmartAudio() override;

    void sendQueuedData(uint32_t maxBytesToSend) override;
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }

    void forwardMessage(const crsf_header_t *message) override;

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {}
    ELRSSerial &_serial;
    void setTXMode() const;
    void setRXMode() const;
};
