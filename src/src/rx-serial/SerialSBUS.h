#pragma once

#include "SerialIO.h"
#include "ELRSSerial.h"

class SerialSBUS final : public SerialIO {
public:
    SerialSBUS(ELRSSerial &port, int8_t txPin, bool invert)
        : SerialIO(&port, &port), streamOut(&port) { port.begin(100000, SERIAL_8E2, -1, txPin, invert); }
    ~SerialSBUS() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {};

    Stream *streamOut;
};
