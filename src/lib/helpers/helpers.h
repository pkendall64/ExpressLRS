#pragma once

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

class NullStream : public Stream
{
  public:
    int available(void)
    {
        return 0;
    }

    void flush(void)
    {
        return;
    }

    int peek(void)
    {
        return -1;
    }

    int read(void)
    {
        return -1;
    }

    size_t write(uint8_t u_Data)
    {
        UNUSED(u_Data);
        return 0x01;
    }

    size_t write(const uint8_t *buffer, size_t size)
    {
        UNUSED(buffer);
        return size;
    }
};

#if defined(PLATFORM_ESP32)
class BufferedStream : public Stream
{
  private:
    Stream *underlying;
    uint8_t buf1[512];
    uint8_t buf2[512];
    uint8_t *buf;
    volatile int current_length;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  public:
    BufferedStream(Stream *underlying)
    {
        current_length = 0;
        buf = buf1;
        this->underlying = underlying;
    }

    int available(void)
    {
        return 0;
    }

    void flush(void)
    {
        portENTER_CRITICAL_SAFE(&mux);
        int len = current_length;
        current_length = 0;
        uint8_t *out = buf;
        buf = (buf == buf1) ? buf2 : buf1;
        portEXIT_CRITICAL_SAFE(&mux);

        underlying->write(out, len);
        return;
    }

    int peek(void)
    {
        return -1;
    }

    int read(void)
    {
        return -1;
    }

    size_t write(uint8_t u_Data)
    {
        portENTER_CRITICAL_SAFE(&mux);
        if (current_length < sizeof(buf1)) buf[current_length++] = u_Data;
        portEXIT_CRITICAL_SAFE(&mux);
        return 0x01;
    }

    size_t write(const uint8_t *buffer, size_t size)
    {
        portENTER_CRITICAL_SAFE(&mux);
        if (size + current_length > sizeof(buf1)) {
            size = sizeof(buf1) - current_length;
        }
        memcpy(buf + current_length, buffer, size);
        current_length += size;
        portEXIT_CRITICAL_SAFE(&mux);
        return size;
    }
};
#endif
