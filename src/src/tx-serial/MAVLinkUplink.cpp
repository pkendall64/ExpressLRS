#include "MAVLinkUplink.h"

#include "common.h"
#include "config.h"
#include "msptypes.h"
#include "stubborn_sender.h"

extern StubbornSender DataUlSender;

namespace
{
constexpr uint32_t UART_INPUT_BUF_LEN = 1024;
FIFO<UART_INPUT_BUF_LEN> uartInputBuffer;
uint8_t mavlinkSSBuffer[CRSF_MAX_PACKET_LEN]{}; // Buffer for current stubborn sender packet (mavlink only)

bool initialize()
{
    return !firmwareOptions.is_airport;
}

int start()
{
    if (config.GetLinkMode() == TX_MAVLINK_MODE) return DURATION_IMMEDIATELY;
    return DURATION_NEVER;
}

int event()
{
    if (connectionState == connected)
    {
        uartInputBuffer.flush();
    }
    return start();
}

int timeout()
{
    if (config.GetLinkMode() == TX_MAVLINK_MODE && !DataUlSender.IsActive())
    {
        const uint16_t count = std::min(uartInputBuffer.size(), (uint16_t)CRSF_PAYLOAD_SIZE_MAX);
        if (count)
        {
            mavlinkSSBuffer[0] = MSP_ELRS_MAVLINK_TLM; // Used on RX to differentiate between std msp opcodes and mavlink
            mavlinkSSBuffer[1] = count;
            // Following n bytes are just raw mavlink
            uartInputBuffer.lock();
            uartInputBuffer.popBytes(mavlinkSSBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, count);
            uartInputBuffer.unlock();
            const uint8_t nextPlayloadSize = count + CRSF_FRAME_NOT_COUNTED_BYTES;
            DataUlSender.SetDataToTransmit(mavlinkSSBuffer, nextPlayloadSize);
        }
    }
    return DURATION_IMMEDIATELY;
}
}

uint16_t MAVLinkUplink::free() const
{
    return uartInputBuffer.free();
}

uint16_t MAVLinkUplink::size() const
{
    return uartInputBuffer.size();
}

void MAVLinkUplink::push(const uint8_t *data, const size_t size)
{
    uartInputBuffer.lock();
    uartInputBuffer.pushBytes(data, size);
    uartInputBuffer.unlock();
}

device_t MAVLinkUplink_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONNECTION_CHANGED | EVENT_CONFIG_MAIN_CHANGED,
};