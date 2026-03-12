#if defined(PLATFORM_ESP32)

#include "hwTimer.h"
#include "logging.h"
#include "driver/gptimer.h"

void (*hwTimer::callbackTick)() = nullptr;
void (*hwTimer::callbackTock)() = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

static gptimer_handle_t timer = nullptr;
static portMUX_TYPE isrMutex = portMUX_INITIALIZER_UNLOCKED;

void hwTimer::init(void (*tick)(), void (*tock)())
{
    if (timer)
        return;

    callbackTick = tick;
    callbackTock = tock;

    constexpr gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    gptimer_new_timer(&config, &timer);

    constexpr gptimer_event_callbacks_t cbs = {
        .on_alarm = callback,
    };

    gptimer_register_event_callbacks(timer, &cbs, nullptr);
    gptimer_enable(timer);

    DBGLN("hwTimer Init");
}

void hwTimer::stop()
{
    if (timer && running)
    {
        running = false;
        gptimer_stop(timer);
        DBGLN("hwTimer stop");
    }
}

void hwTimer::resume()
{
    if (!timer || running)
        return;

#if defined(TARGET_TX)
    const uint32_t interval = HWtimerInterval;
    constexpr bool reload = true;
#else
    constexpr uint32_t interval = 1;   // trigger ASAP
    constexpr bool reload = false;
    isTick = false;
#endif

    gptimer_alarm_config_t alarm = {
        .alarm_count = interval,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = reload
        }
    };

    gptimer_set_alarm_action(timer, &alarm);
    gptimer_start(timer);
    running = true;
    DBGLN("hwTimer resume");
}

void hwTimer::updateInterval(const uint32_t time)
{
    HWtimerInterval = time;
    const gptimer_alarm_config_t alarm = {
        .alarm_count = HWtimerInterval,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    gptimer_set_alarm_action(timer, &alarm);
    DBGLN("hwTimer interval: %d", time);
}

void hwTimer::phaseShift(const int32_t newPhaseShift)
{
    const int32_t minVal = -(HWtimerInterval >> 2);
    const int32_t maxVal = (HWtimerInterval >> 2);
    PhaseShift = constrain(newPhaseShift, minVal, maxVal);
}

bool IRAM_ATTR hwTimer::callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *)
{
    if (!running)
        return false;

    portENTER_CRITICAL_ISR(&isrMutex);

#if defined(TARGET_TX)
    callbackTock();
#else
    uint32_t NextInterval = (HWtimerInterval >> 1) + FreqOffset;

    if (!isTick)
    {
        NextInterval += PhaseShift;
        PhaseShift = 0;
    }

    const gptimer_alarm_config_t alarm = {
        .alarm_count = edata->count_value + NextInterval,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = false,
        }
    };
    gptimer_set_alarm_action(timer, &alarm);

    if (isTick)
    {
        callbackTick();
    }
    else
    {
        callbackTock();
    }
    isTick = !isTick;
#endif

    portEXIT_CRITICAL_ISR(&isrMutex);
    return true;
}

#endif