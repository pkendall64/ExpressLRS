#pragma once

#include "FIFO.h"
#include "crsf_protocol.h"
#include "stubborn_sender.h"

class SerialUplink
{
public:
    void flush() { uartInputBuffer.flush(); }
    uint16_t free() const { return uartInputBuffer.free(); }
    uint16_t size() const { return uartInputBuffer.size(); }
    void push(const uint8_t *data, size_t size);
    void pump(StubbornSender &sender);

private:
    static constexpr uint32_t UART_INPUT_BUF_LEN = 1024;
    FIFO<UART_INPUT_BUF_LEN> uartInputBuffer;
    uint8_t mavlinkSSBuffer[CRSF_MAX_PACKET_LEN]{}; // Buffer for current stubborn sender packet (mavlink only)
};
