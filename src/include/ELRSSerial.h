#pragma once

#if defined(__cplusplus) && defined(TARGET_RX)

#include <HardwareSerial.h>

#if defined(PLATFORM_ESP32)
#include <driver/uart.h>
#include <esp32-hal-matrix.h>
#endif

class ELRSSerial final : public HardwareSerial
{
public:
    using HardwareSerial::begin;

    explicit ELRSSerial(uint8_t uartNumber) : HardwareSerial(uartNumber)
    {
#if defined(PLATFORM_ESP32)
        HardwareSerial::end();
#endif
    }

    bool begin(unsigned long baud, SerialConfig config, int8_t rxPin, int8_t txPin, bool invert)
    {
#if defined(PLATFORM_ESP32)
        const bool halfDuplex = rxPin == txPin && rxPin >= 0;
        _halfDuplexPin = -1;

        if (_uart != nullptr)
        {
            HardwareSerial::end();
        }

        if (!invert)
        {
            uart_set_line_inverse(_uart_nr, UART_SIGNAL_INV_DISABLE);
        }
        _uart = uartBegin(_uart_nr, baud, config, rxPin, txPin, _rxBufferSize, _txBufferSize, invert, 112);
        if (_uart == nullptr)
        {
            return false;
        }

        uartSetRxTimeout(_uart, _rxTimeout);
        const uint8_t fifoFull = _rxFIFOFull ? _rxFIFOFull : (baud > 57600 ? 120 : 1);
        uartSetRxFIFOFull(_uart, fifoFull);
        _rxFIFOFull = fifoFull;
        if (halfDuplex)
        {
            _halfDuplexPin = rxPin;
            setHalfDuplexReceive();
        }
#else
        if ((rxPin != -1 && rxPin != 3) || (txPin != -1 && txPin != 1) || (rxPin == -1 && txPin == -1))
        {
            return false;
        }

        const SerialMode mode = rxPin == -1 ? SERIAL_TX_ONLY : txPin == -1 ? SERIAL_RX_ONLY : SERIAL_FULL;
        HardwareSerial::begin(baud, config, mode, 1, invert);
#endif
        return true;
    }

#if defined(PLATFORM_ESP32)
    bool beginHalfDuplex(unsigned long baud, SerialConfig config, int8_t pin)
    {
        return pin >= 0 && begin(baud, config, pin, pin, false);
    }

    bool isHalfDuplex() const { return _halfDuplexPin >= 0; }

    void setHalfDuplexTransmit()
    {
        if (_halfDuplexPin < 0)
        {
            return;
        }

        pinMode(_halfDuplexPin, OUTPUT);
        digitalWrite(_halfDuplexPin, HIGH);
        pinMatrixOutAttach(_halfDuplexPin, halfDuplexTxSignal(), false, false);
    }

    void setHalfDuplexReceive()
    {
        if (_halfDuplexPin < 0)
        {
            return;
        }

        pinMode(_halfDuplexPin, INPUT_PULLUP);
        pinMatrixInAttach(_halfDuplexPin, halfDuplexRxSignal(), false);
    }

private:
    uint8_t halfDuplexTxSignal() const
    {
#if SOC_UART_NUM > 2
        return _uart_nr == 0 ? U0TXD_OUT_IDX : _uart_nr == 1 ? U1TXD_OUT_IDX : U2TXD_OUT_IDX;
#else
        return _uart_nr == 0 ? U0TXD_OUT_IDX : U1TXD_OUT_IDX;
#endif
    }

    uint8_t halfDuplexRxSignal() const
    {
#if SOC_UART_NUM > 2
        return _uart_nr == 0 ? U0RXD_IN_IDX : _uart_nr == 1 ? U1RXD_IN_IDX : U2RXD_IN_IDX;
#else
        return _uart_nr == 0 ? U0RXD_IN_IDX : U1RXD_IN_IDX;
#endif
    }

    int8_t _halfDuplexPin = -1;
#endif
};

extern ELRSSerial Serial;
#if defined(PLATFORM_ESP32)
extern ELRSSerial Serial1;
#endif
#endif
