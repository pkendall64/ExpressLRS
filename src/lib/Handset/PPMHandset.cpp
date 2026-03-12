#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_TX)

#include "PPMHandset.h"
#include "OTA.h"
#include "crsf_protocol.h"
#include "logging.h"

#include <driver/rmt_rx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

constexpr uint32_t RMT_RESOLUTION_HZ = 4000000;  // 4 MHz, 1 tick = 250 ns (legacy: 80MHz/20)
constexpr uint32_t RMT_TICKS_PER_US = 4;         // 1 us = 4 ticks at 4 MHz

bool PPMHandset::recvDoneCb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    auto *self = static_cast<PPMHandset *>(user_ctx);
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

void PPMHandset::Begin()
{
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
    recv_cfg.signal_range_max_ns = 4000000;  // > 4 ms gap = end of PPM frame
    rmt_receive(rx_channel, recv_buf, RECV_BUF_SYMBOLS * sizeof(rmt_symbol_word_t), &recv_cfg);

    lastPPM = 0;

    if (connected) {
        connected();
    }
}

void PPMHandset::End()
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

void PPMHandset::handleInput()
{
    const auto now = millis();
    size_t count = 0;

    if (xQueueReceive(recv_queue, &count, 0) == pdTRUE && copy_buf && count > 0)
    {
        int channelCount = 0;
        uint32_t localChannelData[CRSF_NUM_CHANNELS];
        const size_t n = count <= RECV_BUF_SYMBOLS ? count : RECV_BUF_SYMBOLS;

        for (size_t i = 0; i < n && channelCount < (int)CRSF_NUM_CHANNELS; i++)
        {
            const rmt_symbol_word_t &item = copy_buf[i];
            if (item.duration0 == 0 || item.duration1 == 0) {
                break;
            }
            channelCount++;
            const auto ppm = (item.duration0 + item.duration1) / RMT_TICKS_PER_US;
            localChannelData[i] = fmap(constrain(ppm, US_CHANNEL_VALUE_STD_MIN, US_CHANNEL_VALUE_STD_MAX), 
                                       US_CHANNEL_VALUE_STD_MIN, US_CHANNEL_VALUE_STD_MAX,
                                       CRSF_CHANNEL_VALUE_STD_MIN, CRSF_CHANNEL_VALUE_STD_MAX);
        }
        numChannels = channelCount;
        lastPPM = now;

        PerformChannelOverrides(localChannelData, numChannels);

        isArmed = numChannels < 5 || CRSF_to_BIT(localChannelData[4]);
        if (channelCount > 0)
        {
            RCDataReceived(localChannelData, numChannels);
        }

        rmt_receive_config_t recv_cfg = {};
        recv_cfg.signal_range_min_ns = 250;
        recv_cfg.signal_range_max_ns = 4000000;
        rmt_receive(rx_channel, recv_buf, RECV_BUF_SYMBOLS * sizeof(rmt_symbol_word_t), &recv_cfg);
    }
    else if (lastPPM && now - 1000 > lastPPM)
    {
        DBGLN("PPM signal lost, disarming");
        isArmed = false;
        if (disconnected)
        {
            disconnected();
        }
        lastPPM = 0;
    }
}

#endif
