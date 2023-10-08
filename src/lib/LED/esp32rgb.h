#include <Arduino.h>

class RgbColor
{
public:
    RgbColor(uint8_t r, uint8_t g, uint8_t b)
        : R(r), G(g), B(b) {}
    explicit RgbColor(uint8_t brightness)
        : R(brightness), G(brightness), B(brightness) {}

    uint8_t R;
    uint8_t G;
    uint8_t B;
};

class ESP32LedDriver
{
public:
    ESP32LedDriver(int count, int pin);
    ~ESP32LedDriver();

    void Begin();
    void Show();
    void ClearTo(RgbColor color, uint16_t first, uint16_t last);
    virtual void SetPixelColor(uint16_t indexPixel, RgbColor color) = 0;

protected:
    uint16_t *out_buffer = nullptr;
    int out_buffer_size;
    int num_leds;
    int gpio_pin;
};

class ESP32LedDriverGRB : public ESP32LedDriver
{
public:
    ESP32LedDriverGRB(int count, int pin)
        : ESP32LedDriver(count, pin) {}
    void SetPixelColor(uint16_t indexPixel, RgbColor color) override;
};

class ESP32LedDriverRGB : public ESP32LedDriver
{
public:
    ESP32LedDriverRGB(int count, int pin)
        : ESP32LedDriver(count, pin) {}
    void SetPixelColor(uint16_t indexPixel, RgbColor color) override;
};
