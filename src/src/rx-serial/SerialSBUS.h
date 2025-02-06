#pragma once

#include "SerialIO.h"

class SerialSBUS final : public SerialIO {
public:
    explicit SerialSBUS(HardwareSerial &stream, const int8_t txPin, const bool invert, const bool _isDjiRsPro) :
        SerialIO(&stream, 100000, SERIAL_8E2, UNDEF_PIN, txPin, invert), isDjiRsPro(_isDjiRsPro) {}

    ~SerialSBUS() override = default;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {}

    const bool isDjiRsPro;
};
