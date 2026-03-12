#include "targets.h"

#if defined(PLATFORM_ESP32)

#include <cstring>
#include "esp32rgb.h"

constexpr uint32_t SAMPLE_RATE = 375000;
constexpr i2s_mclk_multiple_t MCLK_MULTIPLE = I2S_MCLK_MULTIPLE_128;

constexpr auto DMA_BUF_LEN = 1024;
constexpr auto DMA_BUF_COUNT = 2;
constexpr auto I2S_BITS_PER_RGB_BIT = I2S_DATA_BIT_WIDTH_16BIT;
constexpr auto BYTES_PER_LED = 24 * (I2S_BITS_PER_RGB_BIT / 8);
constexpr auto MAX_LEDS = (DMA_BUF_LEN * DMA_BUF_COUNT - 4) / BYTES_PER_LED;

ESP32LedDriver::ESP32LedDriver(const int count, const int pin) : num_leds(count), gpio_pin(pin)
{
    num_leds = std::min(count, MAX_LEDS);
    out_buffer_size = num_leds * BYTES_PER_LED;
    out_buffer = static_cast<uint16_t *>(heap_caps_malloc(out_buffer_size, MALLOC_CAP_DMA));
    memset(out_buffer, 0, out_buffer_size);
}

ESP32LedDriver::~ESP32LedDriver()
{
    if (tx_handle) {
        i2s_channel_disable(tx_handle);
        i2s_del_channel(tx_handle);
        tx_handle = nullptr;
    }
    heap_caps_free(out_buffer);
}

void ESP32LedDriver::Begin() const
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = DMA_BUF_LEN;
    chan_cfg.auto_clear = true;
    if (i2s_new_channel(&chan_cfg, &tx_handle, nullptr) != ESP_OK) {
        return;
    }

    i2s_std_config_t std_cfg = {};
    std_cfg.clk_cfg.sample_rate_hz = SAMPLE_RATE;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.mclk_multiple = MCLK_MULTIPLE;
    std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.ws = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.dout = static_cast<gpio_num_t>(gpio_pin);
    std_cfg.gpio_cfg.din = I2S_GPIO_UNUSED;

    if (i2s_channel_init_std_mode(tx_handle, &std_cfg) != ESP_OK) {
        i2s_del_channel(tx_handle);
        tx_handle = nullptr;
        return;
    }
    i2s_channel_enable(tx_handle);
    delay(1);
}

void ESP32LedDriver::Show() const
{
    if (!tx_handle) {
        return;
    }
    size_t bytes_written = 0;
    i2s_channel_disable(tx_handle);
    i2s_channel_enable(tx_handle);
    i2s_channel_write(tx_handle, out_buffer, out_buffer_size, &bytes_written, 1000);
}

void ESP32LedDriver::ClearTo(const RgbColor color, const int first, const int last)
{
    for (auto i=first ; i<=std::max(last, num_leds-1); i++)
    {
        SetPixelColor(i, color);
    }
}

static const int bit_order[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

void ESP32LedDriverGRB::SetPixelColor(const int indexPixel, const RgbColor color)
{
    if (indexPixel < num_leds)
    {
        const auto loc = indexPixel * 24;
        for(auto bit_pos = 0 ; bit_pos < 8 ; bit_pos++)
        {
            const auto bit = bit_order[bit_pos];
            out_buffer[loc + bit_pos + 0] = (color.G & bit) ? 0xFFE0 : 0xF000;
            out_buffer[loc + bit_pos + 8] = (color.R & bit) ? 0xFFE0 : 0xF000;
            out_buffer[loc + bit_pos + 16] = (color.B & bit) ? 0xFFE0 : 0xF000;
        }
    }
}

void ESP32LedDriverRGB::SetPixelColor(const int indexPixel, const RgbColor color)
{
    if (indexPixel < num_leds)
    {
        const auto loc = indexPixel * 24;
        for(auto bit_pos = 0 ; bit_pos < 8 ; bit_pos++)
        {
            const auto bit = bit_order[bit_pos];
            out_buffer[loc + bit_pos + 0] = (color.R & bit) ? 0xFFE0 : 0xF000;
            out_buffer[loc + bit_pos + 8] = (color.G & bit) ? 0xFFE0 : 0xF000;
            out_buffer[loc + bit_pos + 16] = (color.B & bit) ? 0xFFE0 : 0xF000;
        }
    }
}

#endif
