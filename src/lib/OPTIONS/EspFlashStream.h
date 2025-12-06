#pragma once

#include "targets.h"

class EspFlashStream final : public Stream
{
public:
    EspFlashStream();
    // Set the starting address to use with seek()s
    void setBaseAddress(size_t base);
    size_t getPosition() const { return _flashOffset + _bufferPos; }
    void setPosition(size_t offset);

    // Stream class overrides
    size_t write(uint8_t) override { return 0; }
    int available() override { return (_bufferPos <= sizeof(_buffer)) ? 1 : 0; }
    int read() override;
    int peek() override;

private:
    WORD_ALIGNED_ATTR uint8_t _buffer[4]{};
    size_t _flashBase = 0;
    size_t _flashOffset = 0;
    uint8_t _bufferPos = 0;

    void fillBuffer();
};