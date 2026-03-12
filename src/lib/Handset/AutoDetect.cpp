#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_TX)

#include "AutoDetect.h"
#include "CRSFHandset.h"
#include "PPMHandset.h"
#include "logging.h"

#include <driver/rmt_rx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <rom/gpio.h>
#include <soc/gpio_periph.h>

constexpr uint32_t RMT_RESOLUTION_HZ = 4000000;
constexpr uint32_t RMT_TICKS_PER_US = 4;

bool AutoDetect::recvDoneCb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    auto *self = static_cast<AutoDetect *>(user_ctx);
    if (!self || !edata || !self->copy_buf || !self->recv_queue) {
        return false;
    }
    size_t n = edata->num_symbols;
    if (n > self->RECV_BUF_SYMBOLS) {
        n = self->RECV_BUF_SYMBOLS;
    }
    memcpy(self->copy_buf, edata->received_symbols, n * sizeof(rmt_symbol_word_t));
    self->copy_count = n;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(self->recv_queue, &self->copy_count, &woken);
    return woken == pdTRUE;
}

void AutoDetect::Begin()
{
    if (GPIO_PIN_RCSIGNAL_RX == U0TXD_GPIO_NUM || GPIO_PIN_RCSIGNAL_RX == U0RXD_GPIO_NUM)
    {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN_RCSIGNAL_RX], PIN_FUNC_GPIO);
        pinMode(GPIO_PIN_RCSIGNAL_RX, INPUT);
        gpio_matrix_in(GPIO_PIN_RCSIGNAL_RX, RMT_SIG_IN0_IDX, false);
    }

    recv_buf = static_cast<rmt_symbol_word_t *>(heap_caps_calloc(RECV_BUF_SYMBOLS, sizeof(rmt_symbol_word_t), MALLOC_CAP_INTERNAL));
    copy_buf = static_cast<rmt_symbol_word_t *>(heap_caps_calloc(RECV_BUF_SYMBOLS, sizeof(rmt_symbol_word_t), MALLOC_CAP_INTERNAL));
    recv_queue = xQueueCreate(1, sizeof(size_t));
    if (!recv_buf || !copy_buf || !recv_queue) {
        if (recv_buf) heap_caps_free(recv_buf);
        if (copy_buf) heap_caps_free(copy_buf);
        if (recv_queue) vQueueDelete(recv_queue);
        recv_buf = nullptr;
        copy_buf = nullptr;
        recv_queue = nullptr;
        return;
    }

    rmt_rx_channel_config_t rx_cfg = {};
    rx_cfg.gpio_num = static_cast<gpio_num_t>(GPIO_PIN_RCSIGNAL_RX);
    rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_cfg.resolution_hz = RMT_RESOLUTION_HZ;
    rx_cfg.mem_block_symbols = RECV_BUF_SYMBOLS;
    if (rmt_new_rx_channel(&rx_cfg, &rx_channel) != ESP_OK) {
        heap_caps_free(recv_buf);
        heap_caps_free(copy_buf);
        vQueueDelete(recv_queue);
        recv_buf = nullptr;
        copy_buf = nullptr;
        recv_queue = nullptr;
        rx_channel = nullptr;
        return;
    }

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = recvDoneCb;
    rmt_rx_register_event_callbacks(rx_channel, &cbs, this);

    rmt_enable(rx_channel);

    rmt_receive_config_t recv_cfg = {};
    recv_cfg.signal_range_min_ns = 250;
    recv_cfg.signal_range_max_ns = 25000;  // 25 µs idle = short gap for detection
    rmt_receive(rx_channel, recv_buf, RECV_BUF_SYMBOLS * sizeof(rmt_symbol_word_t), &recv_cfg);

    input_detect = 0;
}

void AutoDetect::End()
{
    if (rx_channel) {
        rmt_disable(rx_channel);
        rmt_del_channel(rx_channel);
        rx_channel = nullptr;
    }
    if (recv_queue) {
        vQueueDelete(recv_queue);
        recv_queue = nullptr;
    }
    if (recv_buf) {
        heap_caps_free(recv_buf);
        recv_buf = nullptr;
    }
    if (copy_buf) {
        heap_caps_free(copy_buf);
        copy_buf = nullptr;
    }
}

void AutoDetect::migrateTo(Handset *that) const
{
    that->setRcChannelsOverrideCallback(RcChannelsOverrideCallback);
    that->setRCDataCallback(RCdataCallback);
    that->registerCallbacks(connected, disconnected);
    that->Begin();
    that->setPacketInterval(RequestedRCpacketInterval);
    handset = that;
    delete this;
}

void AutoDetect::startPPM() const
{
    const_cast<AutoDetect *>(this)->End();
    migrateTo(new PPMHandset());
}

void AutoDetect::startCRSF() const
{
    const_cast<AutoDetect *>(this)->End();

    if (GPIO_PIN_RCSIGNAL_RX == U0TXD_GPIO_NUM || GPIO_PIN_RCSIGNAL_RX == U0RXD_GPIO_NUM)
    {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_PIN_RCSIGNAL_RX], 0);
        Serial.begin(115200);
    }
    migrateTo(new CRSFHandset());
}

void AutoDetect::handleInput()
{
    size_t count = 0;
    const auto now = millis();

    if (xQueueReceive(recv_queue, &count, 0) == pdTRUE && copy_buf && rx_channel)
    {
        lastDetect = now;
        if ((count == 1 && copy_buf[0].duration0 == 0 && copy_buf[0].duration1 == 0) || count == 0)
        {
            input_detect++;
            if (input_detect > 100)
            {
                DBGLN("PPM signal detected");
                startPPM();
                return;
            }
        }
        else
        {
            input_detect--;
            if (input_detect < -100)
            {
                DBGLN("Serial signal detected");
                startCRSF();
                return;
            }
        }

        rmt_receive_config_t recv_cfg = {};
        recv_cfg.signal_range_min_ns = 250;
        recv_cfg.signal_range_max_ns = 25000;
        rmt_receive(rx_channel, recv_buf, RECV_BUF_SYMBOLS * sizeof(rmt_symbol_word_t), &recv_cfg);
    }
    else
    {
        if (now - 1000 > lastDetect && input_detect != 0)
        {
            DBGLN("No signal detected");
            input_detect = 0;
        }
    }
}

#endif
