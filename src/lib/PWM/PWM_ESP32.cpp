#include "PWM.h"

#if defined(PLATFORM_ESP32)
#include <driver/ledc.h>

#include "logging.h"

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
#define SPEED_MODE LEDC_LOW_SPEED_MODE
#else
#define SPEED_MODE LEDC_HIGH_SPEED_MODE
#endif

#define LEDC_CHANNELS SOC_LEDC_CHANNEL_NUM

#define MCPWM_CHANNEL_FLAG 0x100
#define LEDC_CHANNEL_FLAG 0x200

#define IS_MCPWM_CHANNEL(ch) (ch & MCPWM_CHANNEL_FLAG)
#define IS_LEDC_CHANNEL(ch) (ch & LEDC_CHANNEL_FLAG)

#define LEDC_CHANNEL(ch) (ch & 0xFF)
#define MCPWM_CHANNEL(ch) (ch & 0xFF)

#if SOC_MCPWM_SUPPORTED
#include <driver/mcpwm_prelude.h>
#define MCPWM_PAIRS (SOC_MCPWM_GROUPS * SOC_MCPWM_OPERATORS_PER_GROUP)
#define MCPWM_CHANNELS (MCPWM_PAIRS * SOC_MCPWM_GENERATORS_PER_OPERATOR)
#define MCPWM_RESOLUTION_HZ 1000000

struct mcpwm_pair_state_t
{
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator_;
    uint32_t frequency; /* 0 = pair unused */
};

static mcpwm_pair_state_t mcpwm_pairs[MCPWM_PAIRS] {};
static mcpwm_cmpr_handle_t mcpwm_comparators[MCPWM_CHANNELS] {};
static mcpwm_gen_handle_t mcpwm_generators[MCPWM_CHANNELS] {};
static uint32_t mcpwm_frequencies[MCPWM_CHANNELS] {}; /* 0 = channel free; else frequency for piggy-back logic */
#endif

static struct
{
    int8_t pin;
    uint8_t resolution_bits;
    int32_t interval;
    bool attached;
    ledc_timer_t tmr_idx;
} ledc_config[LEDC_CHANNELS];
static uint32_t ledcTimerConfigs[LEDC_TIMER_MAX] {};

/*
 * Modified versions of the ledcSetup/ledcAttachPin from Arduino ESP32 Hal which allows
 * control of the timer to use rather than being directly associated with the channel. This
 * allows multiple channels to use the same timer within a LEDC group (high/low speed) when
 * they are set to the same frequency.
 */
static void ledcSetupEx(const ledc_timer_t timer, const uint32_t freq, uint8_t bit_num)
{
    const ledc_timer_config_t ledc_timer {
        .speed_mode = SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)bit_num,
        .timer_num = timer,
        .freq_hz = freq,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    if (ledc_timer_config(&ledc_timer) != ESP_OK)
    {
        ERRLN("ledc setup failed!");
    }
}

static void ledcAttachPinEx(const uint8_t pin, const ledc_channel_t chan, const ledc_timer_t timer)
{
    const ledc_channel_config_t ledc_channel {
        .gpio_num = pin,
        .speed_mode = SPEED_MODE,
        .channel = chan,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (timer),
        .duty = 0,
        .hpoint = 0,
    };
    const auto err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK)
    {
        ERRLN("ledc_channel_config failed with error 0x%x on pin %d", err, pin);
    }
}

#if SOC_MCPWM_SUPPORTED
static int mcpwm_pair_index(int ch) { return ch / SOC_MCPWM_GENERATORS_PER_OPERATOR; }
static int mcpwm_group_id(int pair) { return pair / CONFIG_SOC_MCPWM_OPERATORS_PER_GROUP; }

static esp_err_t mcpwm_alloc_channel(const int channel, const uint8_t pin, const uint32_t frequency)
{
    const int pair = mcpwm_pair_index(channel);
    mcpwm_pair_state_t *pair_state = &mcpwm_pairs[pair];
    const int group_id = mcpwm_group_id(pair);
    esp_err_t err;

    if (pair_state->frequency == 0)
    {
        /* First use of this pair: create timer and operator */
        mcpwm_timer_config_t timer_config {};
        timer_config.group_id = group_id;
        timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timer_config.resolution_hz = MCPWM_RESOLUTION_HZ;
        timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        timer_config.period_ticks = MCPWM_RESOLUTION_HZ / frequency;
        err = mcpwm_new_timer(&timer_config, &pair_state->timer);
        if (err != ESP_OK)
            return err;

        mcpwm_operator_config_t op_config {};
        op_config.group_id = group_id;
        err = mcpwm_new_operator(&op_config, &pair_state->operator_);
        if (err != ESP_OK)
        {
            mcpwm_del_timer(pair_state->timer);
            pair_state->timer = nullptr;
            return err;
        }
        err = mcpwm_operator_connect_timer(pair_state->operator_, pair_state->timer);
        if (err != ESP_OK)
        {
            mcpwm_del_operator(pair_state->operator_);
            mcpwm_del_timer(pair_state->timer);
            pair_state->operator_ = nullptr;
            pair_state->timer = nullptr;
            return err;
        }
        pair_state->frequency = frequency;
    }
    else if (pair_state->frequency != frequency)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* Create comparator and generator for this channel */
    mcpwm_comparator_config_t cmp_config {};
    err = mcpwm_new_comparator(pair_state->operator_, &cmp_config, &mcpwm_comparators[channel]);
    if (err != ESP_OK)
        return err;

    const mcpwm_generator_config_t gen_config {
        .gen_gpio_num = pin,
    };
    err = mcpwm_new_generator(pair_state->operator_, &gen_config, &mcpwm_generators[channel]);
    if (err != ESP_OK)
    {
        mcpwm_del_comparator(mcpwm_comparators[channel]);
        mcpwm_comparators[channel] = nullptr;
        return err;
    }

    const mcpwm_gen_handle_t gen = mcpwm_generators[channel];
    const mcpwm_cmpr_handle_t cmp = mcpwm_comparators[channel];
    err = mcpwm_generator_set_action_on_timer_event(gen,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK)
        goto cleanup;
    err = mcpwm_generator_set_action_on_compare_event(gen,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK)
        goto cleanup;

    err = mcpwm_comparator_set_compare_value(cmp, 0);
    if (err != ESP_OK)
        goto cleanup;

    if (pair_state->frequency == frequency)
    {
        /* Timer may already be running (piggy-back); only enable/start if first channel of pair */
        const int other = (channel & 1) ? channel - 1 : channel + 1;
        if (mcpwm_frequencies[other] == 0)
        {
            err = mcpwm_timer_enable(pair_state->timer);
            if (err != ESP_OK)
                goto cleanup;
            err = mcpwm_timer_start_stop(pair_state->timer, MCPWM_TIMER_START_NO_STOP);
            if (err != ESP_OK)
            {
                mcpwm_timer_disable(pair_state->timer);
                goto cleanup;
            }
        }
    }

    return ESP_OK;
cleanup:
    mcpwm_del_generator(mcpwm_generators[channel]);
    mcpwm_del_comparator(mcpwm_comparators[channel]);
    mcpwm_generators[channel] = nullptr;
    mcpwm_comparators[channel] = nullptr;
    return err;
}
#endif

pwm_channel_t PWMController::allocate(const uint8_t pin, const uint32_t frequency)
{
#if SOC_MCPWM_SUPPORTED
    /* 1a. Piggy-back: same frequency already in use, partner slot free */
    int channel = -1;
    for (int i = 0; i < MCPWM_CHANNELS; i++)
    {
        if (mcpwm_frequencies[i] == frequency)
        {
            if (i % 2 == 0 && mcpwm_frequencies[i + 1] == 0)
            {
                channel = i + 1;
                break;
            }
            if (i % 2 == 1 && mcpwm_frequencies[i - 1] == 0)
            {
                channel = i - 1;
                break;
            }
        }
    }
    if (channel == -1)
    {
        /* 1b. New pair: both slots of a pair free */
        for (int i = 0; i < MCPWM_CHANNELS; i += 2)
        {
            if (mcpwm_frequencies[i] == 0 && mcpwm_frequencies[i + 1] == 0)
            {
                channel = i;
                break;
            }
        }
    }
    if (channel != -1)
    {
        esp_err_t err = mcpwm_alloc_channel(channel, pin, frequency);
        if (err == ESP_OK)
        {
            mcpwm_frequencies[channel] = frequency;
            DBGLN("allocate mcpwm ch %d on pin %d at %u Hz", channel, pin, frequency);
            return channel | MCPWM_CHANNEL_FLAG;
        }
        DBGLN("mcpwm_alloc_channel %d failed 0x%x", channel, err);
    }
#endif
    // 2. try for a LEDC channel
    for (auto ch = 0; ch < LEDC_CHANNELS; ch++)
    {
        if (ledc_config[ch].resolution_bits == 0)
        {
            uint8_t bits = 0;
            uint32_t clock = 80000000U / frequency; // APB clk src is 80Mhz
            while (clock >>= 1)
            {
                ++bits;
            }
            if (bits >= LEDC_TIMER_BIT_MAX)
            {
                bits = LEDC_TIMER_BIT_MAX - 1;
            }
            for (auto timer_idx = 0; timer_idx < LEDC_TIMER_MAX; timer_idx++)
            {
                if (ledcTimerConfigs[timer_idx] == 0)
                {
                    ledcTimerConfigs[timer_idx] = frequency;
                    ledcSetupEx((ledc_timer_t)timer_idx, frequency, bits);
                }
                if (ledcTimerConfigs[timer_idx] == frequency)
                {
                    ledcAttachPinEx(pin, (ledc_channel_t)ch, (ledc_timer_t)timer_idx);
                    ledc_config[ch].pin = pin;
                    ledc_config[ch].resolution_bits = bits;
                    ledc_config[ch].interval = 1000000U / frequency;
                    ledc_config[ch].attached = true;
                    ledc_config[ch].tmr_idx = (ledc_timer_t)timer_idx;
                    DBGLN("allocate ledc_ch %d on pin %d using ledc_tim: %d, bits: %d", ch, pin, timer_idx, bits);
                    return ch | LEDC_CHANNEL_FLAG;
                }
            }
            break;
        }
    }

    // 3. bail out, nothing left
    DBGLN("No MCPWM or LEDC channels available for frequency %u Hz", frequency);
    return -1;
}

void PWMController::release(const pwm_channel_t channel)
{
    if (IS_LEDC_CHANNEL(channel))
    {
        auto ch = LEDC_CHANNEL(channel);
        ledc_stop(SPEED_MODE, (ledc_channel_t)ch, 0);
        ledc_config[ch].pin = -1;
        ledc_config[ch].resolution_bits = 0;
        ledc_config[ch].interval = 0;
        ledc_config[ch].attached = false;
    }
#if SOC_MCPWM_SUPPORTED
    else if (IS_MCPWM_CHANNEL(channel))
    {
        auto ch = MCPWM_CHANNEL(channel);
        if (ch < MCPWM_CHANNELS && mcpwm_comparators[ch])
        {
            if (mcpwm_generators[ch])
                mcpwm_del_generator(mcpwm_generators[ch]);
            if (mcpwm_comparators[ch])
                mcpwm_del_comparator(mcpwm_comparators[ch]);
            mcpwm_generators[ch] = nullptr;
            mcpwm_comparators[ch] = nullptr;
            mcpwm_frequencies[ch] = 0;

            const int pair = mcpwm_pair_index(ch);
            const int other = (ch & 1) ? ch - 1 : ch + 1;
            if (mcpwm_frequencies[other] == 0)
            {
                mcpwm_pair_state_t *ps = &mcpwm_pairs[pair];
                if (ps->timer)
                {
                    mcpwm_timer_start_stop(ps->timer, MCPWM_TIMER_STOP_EMPTY);
                    mcpwm_timer_disable(ps->timer);
                    mcpwm_del_operator(ps->operator_);
                    mcpwm_del_timer(ps->timer);
                    ps->timer = nullptr;
                    ps->operator_ = nullptr;
                    ps->frequency = 0;
                }
            }
        }
    }
#endif
    else
    {
        ERRLN("Invalid PWM channel %x", channel);
    }
}

static void setLedcMapped(const pwm_channel_t channel, const bool active, const uint32_t raw)
{
    const auto ch = LEDC_CHANNEL(channel);

    if (active)
    {
        if (!ledc_config[ch].attached)
        {
            ledcAttachPinEx(ledc_config[ch].pin, (ledc_channel_t)ch, ledc_config[ch].tmr_idx);
            ledc_config[ch].attached = true;
        }

        ledc_set_duty(SPEED_MODE, (ledc_channel_t)ch, raw);
        ledc_update_duty(SPEED_MODE, (ledc_channel_t)ch);
    }
    else
    {
        if (ledc_config[ch].attached)
        {
            ledc_stop(SPEED_MODE, (ledc_channel_t)ch, 0);
            ledc_config[ch].attached = false;
        }
    }
}

void PWMController::setDuty(const pwm_channel_t channel, const uint16_t duty)
{
    if (IS_LEDC_CHANNEL(channel))
    {
        const auto ch = LEDC_CHANNEL(channel);
        const int32_t maxRaw = (1 << ledc_config[ch].resolution_bits) - 1;
        const int32_t raw = map(duty, 0, 1000, 0, maxRaw);
        setLedcMapped(channel, duty > 0, raw);
    }
#if SOC_MCPWM_SUPPORTED
    else if (IS_MCPWM_CHANNEL(channel))
    {
        const auto ch = MCPWM_CHANNEL(channel);
        if (ch < MCPWM_CHANNELS && mcpwm_comparators[ch])
        {
            const int pair = mcpwm_pair_index(ch);
            const int32_t period_ticks = MCPWM_RESOLUTION_HZ / mcpwm_pairs[pair].frequency;
            const auto compare_ticks = (uint32_t)map(duty, 0, 1000, 0, period_ticks);
            mcpwm_comparator_set_compare_value(mcpwm_comparators[ch], compare_ticks);
        }
    }
#endif
}

void PWMController::setMicroseconds(const pwm_channel_t channel, const uint16_t microseconds)
{
    if (IS_LEDC_CHANNEL(channel))
    {
        const auto ch = LEDC_CHANNEL(channel);
        const int32_t maxRaw = (1 << ledc_config[ch].resolution_bits) - 1;
        const int32_t raw = map(microseconds, 0, ledc_config[ch].interval, 0, maxRaw);
        setLedcMapped(channel, microseconds > 0, raw);
    }
#if SOC_MCPWM_SUPPORTED
    else if (IS_MCPWM_CHANNEL(channel))
    {
        const auto ch = MCPWM_CHANNEL(channel);
        if (ch < MCPWM_CHANNELS && mcpwm_comparators[ch])
            mcpwm_comparator_set_compare_value(mcpwm_comparators[ch], microseconds);
    }
#endif
}

void PWMController::feedWatchdog()
{
}

#endif
