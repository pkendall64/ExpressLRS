#if defined(PLATFORM_ESP32)
#include "hwTimer.h"
#include "logging.h"

void (*hwTimer::callbackTick)() = nullptr;
void (*hwTimer::callbackTock)() = nullptr;

volatile bool hwTimer::running = false;
volatile bool hwTimer::isTick = false;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile int32_t hwTimer::PhaseShift = 0;
volatile int32_t hwTimer::FreqOffset = 0;

// Internal implementation specific variables
static hw_timer_t *timer = nullptr;
static portMUX_TYPE isrMutex = portMUX_INITIALIZER_UNLOCKED;

#if defined(TARGET_RX)
#define HWTIMER_TICKS_PER_US 5
#else
#define HWTIMER_TICKS_PER_US 1
#endif

void ICACHE_RAM_ATTR hwTimer::init(void (*callbackTick)(), void (*callbackTock)())
{
    if (!timer)
    {
        hwTimer::callbackTick = callbackTick;
        hwTimer::callbackTock = callbackTock;
        timer = timerBegin(APB_CLK_FREQ / 1000000 / HWTIMER_TICKS_PER_US);
        timerStop(timer);
        timerAttachInterrupt(timer, hwTimer::callback);
        DBGLN("hwTimer Init");
    }
}

void ICACHE_RAM_ATTR hwTimer::stop()
{
    if (timer && running)
    {
        running = false;
        timerStop(timer);
        DBGLN("hwTimer stop");
    }
}

void ICACHE_RAM_ATTR hwTimer::resume()
{
    if (timer && !running)
    {
        // The timer must be restarted so that the new timerAlarm() period is set.
        timerRestart(timer);
#if defined(TARGET_TX)
        timerAlarm(timer, HWtimerInterval, false, 0);
#else
        // We want the timer to fire tock() ASAP after enabling
        // tock() should always be the first event to maintain consistency
        isTick = false;
        // When using EDGE triggered timer, enabling the timer causes an edge so the interrupt
        // is fired immediately
        // Unlike the 8266 timer, the ESP32 timer can be started without delay.
        // It does not interrupt the currently running IsrCallback(), but triggers immediately once it has completed.
        timerAlarm(timer, 0 * HWTIMER_TICKS_PER_US, false, 0);
#endif
        running = true;
        DBGLN("hwTimer resume");
    }
}

void ICACHE_RAM_ATTR hwTimer::updateInterval(uint32_t time)
{
    // timer should not be running when updateInterval() is called
    HWtimerInterval = time * HWTIMER_TICKS_PER_US;
    if (timer)
    {
        DBGLN("hwTimer interval: %d", time);
        timerWrite(timer, HWtimerInterval);
    }
}

void ICACHE_RAM_ATTR hwTimer::phaseShift(int32_t newPhaseShift)
{
    int32_t minVal = -(HWtimerInterval >> 2);
    int32_t maxVal = (HWtimerInterval >> 2);

    // phase shift is in microseconds
    PhaseShift = constrain(newPhaseShift, minVal, maxVal) * HWTIMER_TICKS_PER_US;
}

void ICACHE_RAM_ATTR hwTimer::callback(void)
{
    if (running)
    {
        portENTER_CRITICAL_ISR(&isrMutex);
#if defined(TARGET_TX)
        callbackTock();
#else
        uint32_t NextInterval = (HWtimerInterval >> 1) + FreqOffset;
        if (hwTimer::isTick)
        {
            timerAlarm(timer, NextInterval, false, 0);
            hwTimer::callbackTick();
        }
        else
        {
            NextInterval += PhaseShift;
            timerAlarm(timer, NextInterval, false, 0);
            PhaseShift = 0;
            hwTimer::callbackTock();
        }
        hwTimer::isTick = !hwTimer::isTick;
#endif
        portEXIT_CRITICAL_ISR(&isrMutex);
    }
}

#endif
