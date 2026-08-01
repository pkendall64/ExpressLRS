#include "SerialUplink.h"

#include "common.h"
#include "config.h"
#include "msptypes.h"

void SerialUplink::push(const uint8_t *data, const size_t size)
{
    uartInputBuffer.lock();
    uartInputBuffer.pushBytes(data, size);
    uartInputBuffer.unlock();
}

void SerialUplink::pump(StubbornSender &sender)
{
    if (config.GetLinkMode() == TX_MAVLINK_MODE && !sender.IsActive())
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
            sender.SetDataToTransmit(mavlinkSSBuffer, nextPlayloadSize);
        }
    }
}