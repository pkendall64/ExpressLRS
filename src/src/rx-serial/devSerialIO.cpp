#include "targets.h"

#if defined(TARGET_RX)

#include "common.h"
#include "device.h"
#include "SerialIO.h"
#include "CRSF.h"
#include "logging.h"

extern SerialIO *serialIO;

static volatile bool frameAvailable = false;

void ICACHE_RAM_ATTR crsfRCFrameAvailable()
{
    frameAvailable = true;
}

static int start()
{
    return DURATION_IMMEDIATELY;
}

static int event()
{
    static connectionState_e lastConnectionState = disconnected;
    serialIO->setFailsafe(connectionState == disconnected && lastConnectionState == connected);
    lastConnectionState = connectionState;
    return DURATION_IGNORE;
}

static int timeout()
{
#if defined(PLATFORM_ESP32)
    // Flush the LOGGING_UART here so all serial IO is done in the same context
    // and does not crash the ESP32. Doing it from ISR and the 2 cores will cause
    // it to crash! So we use a buffer stream and flush here.
    LOGGING_UART.flush();
#endif

    if (connectionState == serialUpdate)
    {
        return DURATION_NEVER;  // stop callbacks when doing serial update
    }

    uint32_t duration = 10; // 10ms callback (i.e. when no theres no model match)
    // only send frames if we have a model match
    if (connectionHasModelMatch)
    {
        duration = serialIO->sendRCFrameToFC(frameAvailable, ChannelData);
    }
    frameAvailable = false;
    // still get telemetry and send link stats if theres no model match
    serialIO->handleUARTout();
    serialIO->handleUARTin();
    return duration;
}

device_t Serial_device = {
    .initialize = nullptr,
    .start = start,
    .event = event,
    .timeout = timeout
};

#endif