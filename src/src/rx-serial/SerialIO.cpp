#include "SerialIO.h"

// ARDUINO_CORE_INVERT_FIX PT1
//
// Code encapsulated by the ARDUINO_CORE_INVERT_FIX #ifdef temporarily fixes EpressLRS issue #2609 which is caused
// by the Arduino core (see https://github.com/espressif/arduino-esp32/issues/9896) and fixed
// by Espressif with Arduino core release 3.0.3 (see https://github.com/espressif/arduino-esp32/pull/9950)
//
// With availability of Arduino core 3.0.3 and upgrading ExpressLRS to Arduino core 3.0.3 the temporary fix
// should be deleted again
//
#define ARDUINO_CORE_INVERT_FIX

#if defined(PLATFORM_ESP32) && defined(ARDUINO_CORE_INVERT_FIX)
#include "driver/uart.h"
#endif
// ARDUINO_CORE_INVERT_FIX PT1 end

SerialIO::SerialIO(HardwareSerial *stream, unsigned long baud, SerialConfig config, int8_t rxPin, int8_t txPin, bool invert)
    : _stream(stream)
{
#if defined(PLATFORM_ESP8266)
    Serial.begin(baud, config, rxPin == UNDEF_PIN ? SERIAL_TX_ONLY : SERIAL_FULL, -1, invert);
#else
    // ARDUINO_CORE_INVERT_FIX PT2
#if defined(ARDUINO_CORE_INVERT_FIX)
    if (invert == false && stream == &Serial)
    {
        uart_set_line_inverse(0, UART_SIGNAL_INV_DISABLE);
    }
#endif
    Serial.begin(baud, config, rxPin == txPin ? UNDEF_PIN : rxPin, txPin, invert);
#endif
}

void SerialIO::setFailsafe(bool failsafe)
{
    this->failsafe = failsafe;
}

void SerialIO::processSerialInput()
{
    auto maxBytes = getMaxSerialReadSize();
    uint8_t buffer[maxBytes];
    auto size = min(_stream->available(), maxBytes);
    _stream->readBytes(buffer, size);
    processBytes(buffer, size);
}

void SerialIO::sendQueuedData(uint32_t maxBytesToSend)
{
    uint32_t bytesWritten = 0;

    while (_fifo.size() > _fifo.peek() && (bytesWritten + _fifo.peek()) < maxBytesToSend)
    {
        _fifo.lock();
        uint8_t OutPktLen = _fifo.pop();
        uint8_t OutData[OutPktLen];
        _fifo.popBytes(OutData, OutPktLen);
        _fifo.unlock();
        noInterrupts();
        this->_stream->write(OutData, OutPktLen); // write the packet out
        interrupts();
        bytesWritten += OutPktLen;
    }
}

void SerialIO::println(const char *str) const
{
    _stream->println(str);
    _stream->flush();
}
