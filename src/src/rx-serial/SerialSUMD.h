#pragma once

#include "SerialIO.h"
#include "crc.h"

class SerialSUMD final : public SerialIO
{
public:
    SerialSUMD(HardwareSerial &stream, int8_t txPin);
    ~SerialSUMD() override = default;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    Crc2Byte crc2Byte {};
    void processBytes(uint8_t *bytes, uint16_t size) override {};
};
