#pragma once

#include "handset.h"

#include "driver/rmt_rx.h"

class AutoDetect final : public Handset
{
public:
    void Begin() override;
    void End() override;
    void handleInput() override;

private:
    static bool recvDoneCb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

    void migrateTo(Handset *that) const;
    void startPPM() const;
    void startCRSF() const;

    int input_detect = 0;
    uint32_t lastDetect = 0;
    rmt_channel_handle_t rx_channel = nullptr;
    QueueHandle_t recv_queue = nullptr;
    rmt_symbol_word_t *recv_buf = nullptr;
    rmt_symbol_word_t *copy_buf = nullptr;
    size_t copy_count = 0;
    static constexpr size_t RECV_BUF_SYMBOLS = 128;
};