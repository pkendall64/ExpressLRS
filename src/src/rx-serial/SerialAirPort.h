#pragma once

#include "FIFO.h"
#include "OTA.h"
#include "SerialIO.h"
#include "options.h"

class SerialAirPort final : public SerialIO {
public:
    explicit SerialAirPort(HardwareSerial &stream, const int8_t rxPin, const int8_t txPin) :
        SerialIO(&stream, firmwareOptions.uart_baud, SERIAL_8N1, rxPin, txPin, false) {}
    ~SerialAirPort() override = default;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    FIFO<AP_MAX_BUF_LEN> apInputBuffer;
    FIFO<AP_MAX_BUF_LEN> apOutputBuffer;

    bool isTlmQueued() const { return apInputBuffer.size() > 0; }
private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;
};
