#include "AirportSerial.h"

#include "FIFO.h"
#include "common.h"
#include "config.h"
#include "options.h"
#include "telemetry_protocol.h"
#include "rxtx_intf.h"

namespace
{
Stream *TxUSB = nullptr;
FIFO<AP_MAX_BUF_LEN> *apInputBuffer;
FIFO<AP_MAX_BUF_LEN> *apOutputBuffer;

bool initialize()
{
    if (!firmwareOptions.is_airport) return false;
#if defined(PLATFORM_ESP8266)
    Serial.begin(firmwareOptions.uart_baud);
    TxUSB = &Serial;
#elif defined(PLATFORM_ESP32_S3)
    // Because we have ARDUINO_USB_MODE enabled, we use USBSerial as the USB device.
    USBSerial.begin(firmwareOptions.uart_baud);
    TxUSB = &USBSerial;
#else
    TxUSB = new HardwareSerial(1);
    ((HardwareSerial *)TxUSB)->begin(firmwareOptions.uart_baud, SERIAL_8N1, U0RXD_GPIO_NUM, U0TXD_GPIO_NUM);
#endif
    apInputBuffer = new FIFO<AP_MAX_BUF_LEN>();
    apOutputBuffer = new FIFO<AP_MAX_BUF_LEN>();
    return true;
}

int start()
{
    config.SetTlm(TLM_RATIO_1_2); // Force TLM ratio of 1:2 for balanced bi-dir link
    config.SetMotionMode(0); // Ensure motion detection is off
    UARTconnected();
    return DURATION_IMMEDIATELY;
}

int event()
{
    if (connectionState == connected)
    {
        apInputBuffer->flush();
        apOutputBuffer->flush();
    }
    return DURATION_IGNORE;
}

int timeout()
{
    auto size = std::min(apInputBuffer->free(), (uint16_t)TxUSB->available());
    if (size > 0)
    {
        uint8_t buf[size];
        size = TxUSB->readBytes(buf, size);
        apInputBuffer->lock();
        apInputBuffer->pushBytes(buf, size);
        apInputBuffer->unlock();
    }
    size = apOutputBuffer->size();
    if (size)
    {
        uint8_t buf[size];
        apOutputBuffer->lock();
        apOutputBuffer->popBytes(buf, size);
        apOutputBuffer->unlock();
        TxUSB->write(buf, size);
    }
    return DURATION_IMMEDIATELY;
}
}


void PackAirportData(OTA_Packet_s * const otaPktPtr)
{
    OtaPackAirportData(otaPktPtr, apInputBuffer);
}

void UnpackAirportData(OTA_Packet_s const * const otaPktPtr)
{
    OtaUnpackAirportData(otaPktPtr, apOutputBuffer);
}

device_t Airport_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONNECTION_CHANGED,
};
