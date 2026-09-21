#pragma once

#include "SerialIO.h"
#include "ELRSSerial.h"
#include "crc.h"

class SerialSUMD final : public SerialIO {
public:
    SerialSUMD(ELRSSerial &port, int8_t txPin) : SerialIO(&port, &port) { port.begin(115200, SERIAL_8N1, -1, txPin, false); crc2Byte.init(16, 0x1021); }
    ~SerialSUMD() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    Crc2Byte crc2Byte {};
    void processBytes(uint8_t *bytes, uint16_t size) override {};
};
