#pragma once

#include "SerialIO.h"

class SerialNOOP final : public SerialIO {
public:
    explicit SerialNOOP() : SerialIO(nullptr, 0, SERIAL_8N1, UNDEF_PIN, UNDEF_PIN, false) {}
    ~SerialNOOP() override = default;

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t* data) override {}
    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return  DURATION_NEVER; }

    void processSerialInput() override {}

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {}
};
