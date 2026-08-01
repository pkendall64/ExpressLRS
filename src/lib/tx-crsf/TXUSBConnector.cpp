#include "TXUSBConnector.h"

#include "common.h"
#include "config.h"

void TXUSBConnector::forwardMessage(const crsf_header_t *message)
{
    if (config.GetLinkMode() != TX_MAVLINK_MODE)
    {
        const uint8_t length = message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES;
        stream->write((uint8_t *)message, length);
    }
}
