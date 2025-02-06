#pragma once

#include "SerialIO.h"
#include "crc.h"

class SerialSUMD final : public SerialIO {
public:
    explicit SerialSUMD(HardwareSerial &stream, const int8_t txPin) :
        SerialIO(&stream, 115200, SERIAL_8N1, UNDEF_PIN, txPin, false) {
        crc2Byte.init(16, 0x1021);
    }
    ~SerialSUMD() override = default;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    Crc2Byte crc2Byte {};
    void processBytes(uint8_t *bytes, uint16_t size) override {};
};
