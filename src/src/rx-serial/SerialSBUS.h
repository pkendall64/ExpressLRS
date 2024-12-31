#pragma once

#include "SerialIO.h"

class SerialSBUS final : public SerialIO {
public:
    explicit SerialSBUS(Stream &out, Stream &in, const bool _isDjiRsPro) : SerialIO(&out, &in), isDjiRsPro(_isDjiRsPro)
    {
        streamOut = &out;
    }

    ~SerialSBUS() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {};

    Stream *streamOut;
    const bool isDjiRsPro;
};
