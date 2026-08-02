#include "TxUSBSerial.h"

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "MAVLink.h"
#include "MAVLinkUplink.h"
#include "TXUSBConnector.h"
#include "common.h"
#include "config.h"
#include "options.h"
#include "rxtx_intf.h"

extern MAVLinkUplink mavLinkUplink;

namespace
{
Stream *TxUSB = nullptr;
TXUSBConnector *usbConnector;
CRSFParser *crsfParser;

bool initialize()
{
    if (firmwareOptions.is_airport) return false;
#if defined(PLATFORM_ESP32_S3)
    // Because we have ARDUINO_USB_MODE enabled, we use USBSerial as the USB device.
    USBSerial.begin(firmwareOptions.uart_baud);
    TxUSB = &USBSerial;
    crsfParser = new CRSFParser();
    return true;
#elif defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
    // If the default UART is not the backpack or for RC, then our UART is available for use
    if(GPIO_PIN_DEBUG_RX != U0RXD_GPIO_NUM && GPIO_PIN_DEBUG_TX != U0TXD_GPIO_NUM && GPIO_PIN_RCSIGNAL_RX != U0RXD_GPIO_NUM && GPIO_PIN_RCSIGNAL_TX != U0TXD_GPIO_NUM)
    {
        TxUSB = new HardwareSerial(1);
        ((HardwareSerial *)TxUSB)->begin(firmwareOptions.uart_baud, SERIAL_8N1, U0RXD_GPIO_NUM, U0TXD_GPIO_NUM);
        crsfParser = new CRSFParser();
        return true;
    }
#endif
    return false;
}

int start()
{
    usbConnector = new TXUSBConnector(TxUSB);
    crsfRouter.addConnector(usbConnector);
    return DURATION_IMMEDIATELY;
}

int timeout()
{
    // If a mavlink packet is received on the USB input, automatically switch the link mode to and process as mavlink
    // Otherwise, USB serial data is processed as CRSF
    const auto size = std::min(mavLinkUplink.free(), (uint16_t)TxUSB->available());
    if (size > 0)
    {
        uint8_t buf[size];
        TxUSB->readBytes(buf, size);

        // If the data is MAVLink, then auto change LinkMode and start the radio link
        // since the user might be operating the module as a standalone unit without a handset.
        if (connectionState == noCrossfire)
        {
            if (isThisAMavPacket(buf, size))
            {
                config.SetLinkMode(TX_MAVLINK_MODE);
                UARTconnected();
            }
        }
        if (config.GetLinkMode() == TX_MAVLINK_MODE)
        {
            mavLinkUplink.push(buf, size);
        }
        else
        {
            crsfParser->processBytes(usbConnector, buf, size);
        }
    }
    return DURATION_IMMEDIATELY;
}
}

void forwardMAVLinkPayloadToUSB(const uint8_t *message, uint8_t size)
{
    if (TxUSB) TxUSB->write(message, size);
}

device_t TxUSBSerial_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
    .subscribe = 0,
};
