#include "SerialIO.h"
#include "common.h"
#include "config.h"
#include "crsf_protocol.h"
#include "device.h"
#include <helpers.h>

#include "SerialAirPort.h"
#include "SerialCRSF.h"
#include "SerialDisplayport.h"
#include "SerialGPS.h"
#include "SerialHoTT_TLM.h"
#include "SerialMavlink.h"
#include "SerialNOOP.h"
#include "SerialSBUS.h"
#include "SerialSUMD.h"
#include "SerialSmartAudio.h"
#include "SerialTramp.h"

static bool pwmSerialDefined = false;

SerialIO *serialIO = nullptr;

#if defined(PLATFORM_ESP32)
SerialIO *serial1IO = nullptr;
#endif

#define NO_SERIALIO_INTERVAL 1000

enum teamraceOutputInhibitState_e
{
    troiPass = 0,            // Allow all packets through, normal operation
    troiDisableAwaitConfirm, // Have received one packet with another model selected, awaiting confirm to Inhibit
    troiInhibit,             // Inhibit all output
    troiEnableAwaitConfirm,  // Have received one packet with this model selected, awaiting confirm to Pass
};

typedef struct devserial_ctx_s
{
    SerialIO **io;
    bool frameAvailable;
    bool frameMissed;
    connectionState_e lastConnectionState;
    uint8_t lastTeamracePosition;
    teamraceOutputInhibitState_e teamraceOutputInhibitState;
} devserial_ctx_t;

static devserial_ctx_t serial0;
#if defined(PLATFORM_ESP32)
static devserial_ctx_t serial1;
#endif

void serialPreConfigure()
{
    // pre-initialise serial must be done before anything as some libs write
    // to the serial port, so they'll block if the buffer fills
#if defined(DEBUG_LOG)
    Serial.begin(firmwareOptions.uart_baud);
    SerialLogger = &Serial;
#else
    SerialLogger = new NullStream();
#endif
}

void ICACHE_RAM_ATTR crsfRCFrameAvailable()
{
    serial0.frameAvailable = true;
#if defined(PLATFORM_ESP32)
    serial1.frameAvailable = true;
#endif
}

void ICACHE_RAM_ATTR crsfRCFrameMissed()
{
    serial0.frameMissed = true;
#if defined(PLATFORM_ESP32)
    serial1.frameMissed = true;
#endif
}

static void setupSerial()
{
#if !defined(DEBUG_CRSF_NO_OUTPUT)
    if (GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN && !pwmSerialDefined)
#endif
    {
// For PWM receivers with no serial pins defined, only turn on the Serial port if logging is on
#if defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)
#if defined(PLATFORM_ESP32_S3) && !defined(ESP32_S3_USB_JTAG_ENABLED)
        // Requires pull-down on GPIO3.  If GPIO3 has a pull-up (for JTAG) this doesn't work.
        USBSerial.begin(serialBaud);
        SerialLogger = &USBSerial;
#else
        Serial.begin(serialBaud);
        SerialLogger = &Serial;
#endif
#else
        SerialLogger = new NullStream();
#endif
        serialIO = new SerialNOOP();
        return;
    }

    if (firmwareOptions.is_airport)
    {
        serialIO = new SerialAirPort(Serial, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX);
    }
    else
    {
        switch (config.GetSerialProtocol())
        {
        case PROTOCOL_CRSF:
            serialIO = new SerialCRSF(Serial, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, false);
            break;
        case PROTOCOL_INVERTED_CRSF:
            serialIO = new SerialCRSF(Serial, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, true);
            break;
        case PROTOCOL_SBUS:
            serialIO = new SerialSBUS(Serial, GPIO_PIN_RCSIGNAL_TX, true, false);
            break;
        case PROTOCOL_INVERTED_SBUS:
            serialIO = new SerialSBUS(Serial, GPIO_PIN_RCSIGNAL_TX, false, false);
            break;
        case PROTOCOL_DJI_RS_PRO:
            serialIO = new SerialSBUS(Serial, GPIO_PIN_RCSIGNAL_TX, true, true);
            break;
        case PROTOCOL_SUMD:
            serialIO = new SerialSUMD(Serial, GPIO_PIN_RCSIGNAL_TX);
            break;
        case PROTOCOL_HOTT_TLM:
            serialIO = new SerialHoTT_TLM(Serial, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX);
            break;
        case PROTOCOL_MSP_DISPLAYPORT:
            serialIO = new SerialDisplayport(Serial, GPIO_PIN_RCSIGNAL_TX);
            break;
        case PROTOCOL_GPS:
            serialIO = new SerialGPS(Serial, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX);
            break;
        case PROTOCOL_MAVLINK:
            serialIO = new SerialMavlink(Serial, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX);
        }
    }

#if defined(DEBUG_ENABLED)
#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    USBSerial.begin(460800);
    SerialLogger = &USBSerial;
#else
    SerialLogger = &Serial;
#endif
#else
    SerialLogger = new NullStream();
#endif
}

void reconfigureSerial0()
{
    SerialLogger = new NullStream();
    if (serialIO != nullptr)
    {
        Serial.end();
        delete serialIO;
        serialIO = nullptr;
    }
    setupSerial();
}

static bool initialize0()
{
    // If serial is not already defined, then see if there is serial pin configured in the PWM configuration
    if (OPT_HAS_SERVO_OUTPUT && GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN)
    {
        for (int i = 0; i < GPIO_PIN_PWM_OUTPUTS_COUNT; i++)
        {
            const auto pinMode = static_cast<eServoOutputMode>(config.GetPwmChannel(i)->val.mode);
            if (pinMode == somSerial)
            {
                pwmSerialDefined = true;
                break;
            }
        }
    }
    setupSerial();
    return true;
}

static int start0()
{
    serial0.io = &serialIO;
    serial0.lastConnectionState = disconnected;
    return DURATION_IMMEDIATELY;
}

#if defined(PLATFORM_ESP32)
static void setupSerial1()
{
    //
    // init secondary serial and protocol
    //
    int8_t serial1RXpin = GPIO_PIN_SERIAL1_RX;
    if (serial1RXpin == UNDEF_PIN)
    {
        for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
        {
            if (config.GetPwmChannel(ch)->val.mode == somSerial1RX)
                serial1RXpin = GPIO_PIN_PWM_OUTPUTS[ch];
        }
    }

    int8_t serial1TXpin = GPIO_PIN_SERIAL1_TX;
    if (serial1TXpin == UNDEF_PIN)
    {
        for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
        {
            if (config.GetPwmChannel(ch)->val.mode == somSerial1TX)
                serial1TXpin = GPIO_PIN_PWM_OUTPUTS[ch];
        }
    }

    switch (config.GetSerial1Protocol())
    {
    case PROTOCOL_SERIAL1_OFF:
        break;
    case PROTOCOL_SERIAL1_CRSF:
        serial1IO = new SerialCRSF(Serial1, serial1RXpin, serial1TXpin, false);
        break;
    case PROTOCOL_SERIAL1_INVERTED_CRSF:
        serial1IO = new SerialCRSF(Serial1, serial1RXpin, serial1TXpin, true);
        break;
    case PROTOCOL_SERIAL1_SBUS:
        serial1IO = new SerialSBUS(Serial1, serial1TXpin, true, false);
        break;
    case PROTOCOL_SERIAL1_DJI_RS_PRO:
        serial1IO = new SerialSBUS(Serial1, serial1TXpin, true, true);
        break;
    case PROTOCOL_SERIAL1_INVERTED_SBUS:
        serial1IO = new SerialSBUS(Serial1, serial1TXpin, false, false);
        break;
    case PROTOCOL_SERIAL1_SUMD:
        serial1IO = new SerialSUMD(Serial1, serial1TXpin);
        break;
    case PROTOCOL_SERIAL1_HOTT_TLM:
        serial1IO = new SerialHoTT_TLM(Serial1, serial1RXpin, serial1TXpin);
        break;
    case PROTOCOL_SERIAL1_TRAMP:
        serial1IO = new SerialTramp(Serial1, serial1TXpin);
        break;
    case PROTOCOL_SERIAL1_SMARTAUDIO:
        serial1IO = new SerialSmartAudio(Serial1, serial1TXpin);
        break;
    case PROTOCOL_SERIAL1_MSP_DISPLAYPORT:
        serial1IO = new SerialDisplayport(Serial1, serial1TXpin);
        break;
    case PROTOCOL_SERIAL1_GPS:
        serial1IO = new SerialGPS(Serial1, serial1TXpin, serial1TXpin);
        break;
    }
}

void reconfigureSerial1()
{
    if (serial1IO != nullptr)
    {
        Serial1.end();
        delete serial1IO;
        serial1IO = nullptr;
    }
    setupSerial1();
}

static bool initialize1()
{
    setupSerial1();
    return true;
}

static int start1()
{
    serial1.io = &serial1IO;
    serial1.lastConnectionState = disconnected;
    return DURATION_IMMEDIATELY;
}
#endif

static void event(devserial_ctx_t *ctx)
{
    if ((*(ctx->io)) != nullptr)
    {
        if (ctx->lastConnectionState != connectionState)
        {
            (*(ctx->io))->setFailsafe(connectionState == disconnected);
        }
    }

    ctx->lastConnectionState = connectionState;
}

static int event0()
{
    event(&serial0);
    return DURATION_IGNORE;
}

#if defined(PLATFORM_ESP32)
static int event1()
{
    event(&serial1);
    return DURATION_IGNORE;
}
#endif

/***
 * @brief: Convert the current TeamraceChannel value to the appropriate config value for comparison
 */
static uint8_t teamraceChannelToConfigValue()
{
    // SWITCH3b is 1,2,3,4,5,6,x,Mid
    //             0 1 2 3 4 5    7
    // Config values are Disabled,1,2,3,Mid,4,5,6
    //                      0     1 2 3  4  5 6 7
    const uint8_t retVal = CRSF_to_SWITCH3b(ChannelData[config.GetTeamraceChannel()]);
    switch (retVal)
    {
    case 0: // passthrough
    case 1: // passthrough
    case 2:
        return retVal + 1;
    case 3: // passthrough
    case 4: // passthrough
    case 5:
        return retVal + 2;
    case 7:
        return 4; // "Mid"
    default:
        // CRSF_to_SWITCH3b should only return 0-5,7 but we must return a value
        return 0;
    }
}

/***
 * @brief: Determine if FrameAvailable and it should be sent to FC
 * @return: TRUE if a new frame is available and should be processed
 */
static bool confirmFrameAvailable(devserial_ctx_t *ctx)
{
    if (!ctx->frameAvailable)
        return false;

    ctx->frameAvailable = false;

    // ModelMatch failure always prevents passing the frame on
    if (!connectionHasModelMatch)
        return false;

    constexpr uint8_t CONFIG_TEAMRACE_POS_OFF = 0;
    if (config.GetTeamracePosition() == CONFIG_TEAMRACE_POS_OFF)
    {
        ctx->teamraceOutputInhibitState = troiPass;
        return true;
    }

    // Pass the packet on if in troiPass (of course) or
    // troiDisableAwaitConfirm (keep sending channels until the teamracepos stabilizes)
    const bool retVal = ctx->teamraceOutputInhibitState < troiInhibit;

    const uint8_t newTeamracePosition = teamraceChannelToConfigValue();

    switch (ctx->teamraceOutputInhibitState)
    {
    case troiPass:
        // User appears to be switching away from this model, wait for confirm
        if (newTeamracePosition != config.GetTeamracePosition())
            ctx->teamraceOutputInhibitState = troiDisableAwaitConfirm;
        break;

    case troiDisableAwaitConfirm:
        // Must receive the same new position twice in a row for state to change
        if (ctx->lastTeamracePosition == newTeamracePosition)
        {
            if (newTeamracePosition != config.GetTeamracePosition())
                ctx->teamraceOutputInhibitState = troiInhibit; // disable output
            else
                ctx->teamraceOutputInhibitState = troiPass; // return to normal
        }
        break;

    case troiInhibit:
        // User appears to be switching to this model, wait for confirm
        if (newTeamracePosition == config.GetTeamracePosition())
            ctx->teamraceOutputInhibitState = troiEnableAwaitConfirm;
        break;

    case troiEnableAwaitConfirm:
        // Must receive the same new position twice in a row for state to change
        if (ctx->lastTeamracePosition == newTeamracePosition)
        {
            if (newTeamracePosition == config.GetTeamracePosition())
                ctx->teamraceOutputInhibitState = troiPass; // return to normal
            else
                ctx->teamraceOutputInhibitState = troiInhibit; // back to disabled
        }
        break;
    }

    ctx->lastTeamracePosition = newTeamracePosition;
    // troiPass or troiDisablePending indicate the model is selected still,
    // however returning true if troiDisablePending means this RX could send
    // telemetry and we do not want that
    teamraceHasModelMatch = ctx->teamraceOutputInhibitState == troiPass;
    return retVal;
}

static int timeout(devserial_ctx_t *ctx)
{
    if (*(ctx->io) == nullptr)
    {
        return NO_SERIALIO_INTERVAL;
    }

    // stop callbacks when serial driver wants immediate sends or when doing serial update
    if ((*(ctx->io))->sendImmediateRC() || connectionState == serialUpdate)
    {
        return DURATION_NEVER;
    }

    /***
     * TODO: This contains a problem!!
     * confirmFrameAvailable() is designed to be the thing that determines if RC frames are to be sent out
     * which includes:
     *      - No data received to be sent (interpacket delay)
     *      - Connection does not have model match
     *      - TeamRace enabled but different position selected
     * However, the SBUS IO writer doesn't go off new data coming in, it just sends data at a 9ms cadence
     * and therefore does not respect any of these conditions, relying on the one-off "failsafe" member
     * modelmatch was addressed in #2211, but resolving the merge conflict here (capnbry) re-breaks it
     *
     * Commiting this anyway though to work out a better resolution
     */

    noInterrupts();
    bool missed = ctx->frameMissed;
    ctx->frameMissed = false;
    interrupts();

    // Verify there is new ChannelData and they should be sent on
    bool sendChannels = confirmFrameAvailable(ctx);

    return (*(ctx->io))->sendRCFrame(sendChannels, missed, ChannelData);
}

void sendImmediateRC()
{
    if (*(serial0.io) != nullptr && (*(serial0.io))->sendImmediateRC() && connectionState != serialUpdate)
    {
        const bool missed = serial0.frameMissed;
        serial0.frameMissed = false;

        // Verify there is new ChannelData and they should be sent on
        const bool sendChannels = confirmFrameAvailable(&serial0);

        (*(serial0.io))->sendRCFrame(sendChannels, missed, ChannelData);
    }
}

void handleSerialIO()
{
    // still get telemetry and send link stats if there's no model match
    if (*(serial0.io) != nullptr)
    {
        (*(serial0.io))->processSerialInput();
        (*(serial0.io))->sendQueuedData((*(serial0.io))->getMaxSerialWriteSize());
    }
#if defined(PLATFORM_ESP32)
    if (*(serial1.io) != nullptr)
    {
        (*(serial1.io))->processSerialInput();
        (*(serial1.io))->sendQueuedData((*(serial1.io))->getMaxSerialWriteSize());
    }
#endif
}

static int timeout0()
{
    return timeout(&serial0);
}

#if defined(PLATFORM_ESP32)
static int timeout1()
{
    return timeout(&serial1);
}
#endif

device_t Serial0_device = {
    .initialize = initialize0,
    .start = start0,
    .event = event0,
    .timeout = timeout0,
    .subscribe = EVENT_CONNECTION_CHANGED};

#if defined(PLATFORM_ESP32)
device_t Serial1_device = {
    .initialize = initialize1,
    .start = start1,
    .event = event1,
    .timeout = timeout1,
    .subscribe = EVENT_CONNECTION_CHANGED};
#endif
