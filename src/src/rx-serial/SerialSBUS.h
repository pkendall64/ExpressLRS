#pragma once

#include "SerialIO.h"

class SerialSBUS final : public SerialIO {
public:
    explicit SerialSBUS(HardwareSerial &stream, const bool _isDjiRsPro) : SerialIO(&stream), isDjiRsPro(_isDjiRsPro) {}

    ~SerialSBUS() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {};

    const bool isDjiRsPro;
};
