#include <targets.h>

#include "BackpackSerial.h"

#include "MAVLink.h"
#include "SerialUplink.h"
#include "config.h"
#include "devBackpack.h"
#include "helpers.h"
#include "logging.h"

extern void UARTconnected();

bool BackpackSerial::initialize()
{
    /*
     * Setup the logging/backpack serial port, we always need a place to send data even if there is no backpack!
     */
#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
    if (GPIO_PIN_DEBUG_RX != UNDEF_PIN && GPIO_PIN_DEBUG_TX != UNDEF_PIN)
    {
        BackpackOrLogStrm = new HardwareSerial(2);
        ((HardwareSerial *)BackpackOrLogStrm)->begin(BACKPACK_LOGGING_BAUD, SERIAL_8N1, GPIO_PIN_DEBUG_RX, GPIO_PIN_DEBUG_TX);
        return true;
    }
#endif
    BackpackOrLogStrm = new NullStream();
    return false;
}

size_t BackpackSerial::available()
{
    return BackpackOrLogStrm->available();
}

void BackpackSerial::poll(SerialUplink &uplink)
{
    // Backpack will not switch modes, but will process data as mavlink if the link mode is already set to mavlink
    // Backpack serial data is ALSO always processed as backpack MSP
    if (BackpackOrLogStrm->available())
    {
        auto size = std::min(uplink.free(), (uint16_t)BackpackOrLogStrm->available());
        if (size > 0)
        {
            uint8_t buf[size];
            BackpackOrLogStrm->readBytes(buf, size);

            // If the TX is in Mavlink mode, push the bytes into the fifo buffer
            if (config.GetLinkMode() == TX_MAVLINK_MODE)
            {
                uplink.push(buf, size);

                // The TX is in MAVLink mode and receiving data from the Backpack,
                // start the radio since the user might be operating the module as a standalone unit without a handset.
                if (connectionState == noCrossfire)
                {
                    if (isThisAMavPacket(buf, size))
                    {
                        UARTconnected();
                    }
                }
            }

            // Try to parse any MSP packets from the Backpack
            ParseMSPData(buf, size);
        }
    }
}