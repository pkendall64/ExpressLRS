#include "targets.h"

#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)

#include "logging.h"
#include "driver/pulse_cnt.h"

#define TACHO_PULSES_PER_REV 4
#define PCNT_HIGH_LIMIT 30000

static pcnt_unit_handle_t pcnt_unit = nullptr;
static pcnt_channel_handle_t pcnt_chan = nullptr;
static uint32_t lastTime = 0;

void init_rpm_counter(const int pin)
{
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -1;
    unit_config.high_limit = PCNT_HIGH_LIMIT;
    unit_config.flags.accum_count = true;

    if (pcnt_new_unit(&unit_config, &pcnt_unit) != ESP_OK) {
        return;
    }

    pcnt_chan_config_t chan_config = {};
    chan_config.edge_gpio_num = pin;
    chan_config.level_gpio_num = -1;  // not used

    if (pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan) != ESP_OK) {
        pcnt_del_unit(pcnt_unit);
        pcnt_unit = nullptr;
        return;
    }

    // Count up on rising edge, hold on falling edge
    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);

    // Watch point at high limit so accumulator counts overflows
    pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT);

    // Glitch filter
    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1250;
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}

uint32_t get_rpm()
{
    if (pcnt_unit == nullptr) {
        return 0;
    }
    int count = 0;
    pcnt_unit_get_count(pcnt_unit, &count);
    pcnt_unit_clear_count(pcnt_unit);
    const uint32_t now = millis();
    const uint32_t elapsed = now - lastTime;
    lastTime = now;
    if (elapsed == 0) {
        return 0;
    }
    const auto total_pulses = abs(count);
    return total_pulses * 60000U / TACHO_PULSES_PER_REV / elapsed;
}

#endif
