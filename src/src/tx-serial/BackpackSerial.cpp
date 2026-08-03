#include <targets.h>

#if defined(PLATFORM_ESP32)
#include "BackpackSerial.h"

#include "Backpack.h"
#include "MAVLink.h"
#include "MAVLinkUplink.h"
#include "config.h"
#include "helpers.h"
#include "hwTimer.h"
#include "logging.h"
#include "msp.h"
#include "msptypes.h"
#include "rxtx_intf.h"

extern MAVLinkUplink mavLinkUplink;

namespace
{
#define GPIO_PIN_BOOT0 0
#define BACKPACK_PERIOD_MS  20

[[noreturn]] void startPassthrough(const bool useUSBSerial)
{
    // stop everything
    devicesStop();
    Radio.End();
    hwTimer::stop();
    handset->End();

    Stream *uplink = &CRSFHandset::Port;

    const uint32_t baud = PASSTHROUGH_BAUD == -1 ? BACKPACK_LOGGING_BAUD : PASSTHROUGH_BAUD;
#if defined(PLATFORM_ESP32_S3)
    if (useUSBSerial)
    {
        uplink = &USBSerial;
        USBSerial.setTxBufferSize(1024);
        USBSerial.setRxBufferSize(16384);
    }
    else
#endif
    {
        CRSFHandset::Port.begin(baud, SERIAL_8N1, U0RXD_GPIO_NUM, U0TXD_GPIO_NUM);
        CRSFHandset::Port.setTxBufferSize(1024);
        CRSFHandset::Port.setRxBufferSize(16384);
    }
    disableLoopWDT();

    const auto backpackUart = (HardwareSerial *)BackpackOrLogStrm;
    if (baud != BACKPACK_LOGGING_BAUD)
    {
        backpackUart->begin(PASSTHROUGH_BAUD, SERIAL_8N1, GPIO_PIN_DEBUG_RX, GPIO_PIN_DEBUG_TX);
    }
    backpackUart->setRxBufferSize(1024);
    backpackUart->setTxBufferSize(16384);

    // reset ESP8285 into bootloader mode
    digitalWrite(GPIO_PIN_BACKPACK_BOOT, HIGH);
    delay(100);
    digitalWrite(GPIO_PIN_BACKPACK_EN, LOW);
    delay(100);
    digitalWrite(GPIO_PIN_BACKPACK_EN, HIGH);
    delay(50);

    uplink->flush();
    backpackUart->flush();

    uint8_t buf[64];
    while (backpackUart->available())
    {
        backpackUart->readBytes(buf, sizeof(buf));
    }

    // go hard!
    for (;;)
    {
        int available_bytes = min(uplink->available(), static_cast<int>(sizeof(buf)));
        auto bytes_read = uplink->readBytes(buf, available_bytes);
        backpackUart->write(buf, bytes_read);

        available_bytes = min(backpackUart->available(), static_cast<int>(sizeof(buf)));
        bytes_read = backpackUart->readBytes(buf, available_bytes);
        uplink->write(buf, bytes_read);
    }
}

int debouncedRead(int pin)
{
    static constexpr uint8_t min_matches = 100;

    static int last_state = -1;
    static uint8_t matches = 0;

    const int current_state = digitalRead(pin);
    if (current_state == last_state)
    {
        matches = min(min_matches, static_cast<uint8_t>(matches + 1));
    }
    else
    {
        // We are bouncing. Reset the match counter.
        matches = 0;
        DBGLN("Bouncing!, current state: %d, last_state: %d, matches: %d", current_state, last_state, matches);
    }

    if (matches == min_matches)
    {
        // We have a stable state and report it.
        return current_state;
    }

    last_state = current_state;

    // We don't have a definitive state we could report.
    return -1;
}

bool initialize()
{
    /*
     * Setup the logging/backpack serial port, we always need a place to send data even if there is no backpack!
     */
#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
    if (OPT_USE_TX_BACKPACK && GPIO_PIN_DEBUG_RX != UNDEF_PIN && GPIO_PIN_DEBUG_TX != UNDEF_PIN)
    {
        BackpackOrLogStrm = new HardwareSerial(2);
        ((HardwareSerial *)BackpackOrLogStrm)->begin(BACKPACK_LOGGING_BAUD, SERIAL_8N1, GPIO_PIN_DEBUG_RX, GPIO_PIN_DEBUG_TX);

        // Initialize the backpack if there is one and we're not in airport mode
        if (GPIO_PIN_BACKPACK_EN != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_BOOT0, INPUT); // setup so we can detect pin-change for passthrough mode
            pinMode(GPIO_PIN_BACKPACK_BOOT, OUTPUT);
            pinMode(GPIO_PIN_BACKPACK_EN, OUTPUT);
            // Shut down the backpack via EN pin and hold it there until the first event()
            digitalWrite(GPIO_PIN_BACKPACK_EN, LOW);   // enable low
            digitalWrite(GPIO_PIN_BACKPACK_BOOT, LOW); // bootloader pin high
            delay(20);
            // Rely on event() to boot
        }

        if (OPT_USE_TX_BACKPACK && !firmwareOptions.is_airport)
        {
            return true;
        }
        return false;
    }
#endif
    BackpackOrLogStrm = new NullStream();
    return false;
}

int start()
{
    resetBackpackChannelData();
    return config.GetBackpackDisable() ? DURATION_NEVER : DURATION_IMMEDIATELY;
}

int event()
{
    const bool disabled = config.GetBackpackDisable() || connectionState == bleJoystick || connectionState == wifiUpdate;
    if (GPIO_PIN_BACKPACK_EN != UNDEF_PIN)
    {
        // EN should be HIGH to be active
        digitalWrite(GPIO_PIN_BACKPACK_EN, disabled ? LOW : HIGH);
    }

    ProcessEvents(disabled);

    return disabled ? DURATION_NEVER : DURATION_IMMEDIATELY;
}

int timeout()
{
    // Backpack will not switch modes, but will process data as mavlink if the link mode is already set to mavlink
    // Backpack serial data is ALSO always processed as backpack MSP
    if (BackpackOrLogStrm->available())
    {
        auto size = std::min(mavLinkUplink.free(), (uint16_t)BackpackOrLogStrm->available());
        if (size > 0)
        {
            uint8_t buf[size];
            BackpackOrLogStrm->readBytes(buf, size);

            // If the TX is in Mavlink mode, push the bytes into the fifo buffer
            if (config.GetLinkMode() == TX_MAVLINK_MODE)
            {
                mavLinkUplink.push(buf, size);

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

    const uint32_t now = millis();
    static uint32_t lastCall = 0;
    if (now - lastCall > BACKPACK_PERIOD_MS)
    {
        lastCall = now;
        ProcessPendingCommands();
    }
    return DURATION_IMMEDIATELY;
}
}

void checkBackpackUpdate()
{
    if (OPT_USE_TX_BACKPACK)
    {
        if (GPIO_PIN_BACKPACK_EN != UNDEF_PIN && debouncedRead(GPIO_PIN_BOOT0) == 0)
        {
            startPassthrough(false);
        }
    }
}

void sendMAVLinkTelemetryToBackpack(const uint8_t *data, const uint8_t count)
{
    if (config.GetBackpackDisable() || config.GetBackpackTlmMode() == BACKPACK_TELEM_MODE_OFF)
    {
        // Backpack telemetry is off
        return;
    }

    BackpackOrLogStrm->write(data, count);
}

void sendCRSFTelemetryToBackpack(const uint8_t *data)
{
    if (config.GetBackpackDisable() || config.GetBackpackTlmMode() == BACKPACK_TELEM_MODE_OFF || config.GetLinkMode() == TX_MAVLINK_MODE)
    {
        return;
    }

    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_CRSF_TLM;

    uint8_t size = CRSF_FRAME_SIZE(data[CRSF_TELEMETRY_LENGTH_INDEX]);
    if (size > CRSF_MAX_PACKET_LEN)
    {
        ERRLN("CRSF frame exceeds max length");
        return;
    }

    for (uint8_t i = 0; i < size; ++i)
    {
        packet.addByte(data[i]);
    }

    MSP::sendPacket(&packet, BackpackOrLogStrm); // send to tx-backpack as MSP
}

void sendMSPToBackpack(const void *packet)
{
    MSP::sendPacket(static_cast<const mspPacket_t *>(packet), BackpackOrLogStrm); // send to tx-backpack as MSP
}

void checkForUpdateSync(const uint8_t *data, const uint16_t count)
{
#if defined(PLATFORM_ESP32_S3)
    // Start passthrough mode if an Espressif resync packet is detected on the USB port
    static const uint8_t resync[] = {
        0xc0,0x00,0x08,0x24,0x00,0x00,0x00,0x00,0x00,0x07,0x07,0x12,0x20,0x55,0x55,0x55,0x55,
        0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55, 0x55,0x55,
        0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0xc0
    };
    static int resync_pos = 0;
    for (int i=0 ; i<count ; i++)
    {
        if (data[i] == resync[resync_pos])
        {
            resync_pos++;
            if (resync_pos == sizeof(resync)) startPassthrough(true);
        }
        else
        {
            resync_pos = 0;
        }
    }
#endif
}

device_t BackpackSerial_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONNECTION_CHANGED | EVENT_CONFIG_MAIN_CHANGED | EVENT_ENTER_BIND_MODE,
};
#endif
