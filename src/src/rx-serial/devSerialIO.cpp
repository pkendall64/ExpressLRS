#include "targets.h"

#if defined(TARGET_RX)

#include "common.h"
#include "config.h"
#include "device.h"
#include "helpers.h"
#include "CRSF.h"

#include "SerialIO.h"
#include "SerialNOOP.h"
#include "SerialCRSF.h"
#include "SerialSBUS.h"
#include "SerialSUMD.h"
#include "SerialAirPort.h"
#include "SerialHoTT_TLM.h"

/* SERIAL_PROTOCOL_TX is used by CRSF output */
#if defined(TARGET_RX_FM30_MINI)
HardwareSerial SERIAL_PROTOCOL_TX(USART2);
#elif defined(TARGET_DIY_900_RX_STM32)
HardwareSerial SERIAL_PROTOCOL_TX(USART1);
#else
#define SERIAL_PROTOCOL_TX Serial
#endif
/* SERIAL_PROTOCOL_RX is used by telemetry receiver and can be on a different peripheral */
#if defined(TARGET_RX_GHOST_ATTO_V1) /* !TARGET_RX_GHOST_ATTO_V1 */
#define SERIAL_PROTOCOL_RX CrsfRxSerial
HardwareSerial CrsfRxSerial(USART1, HALF_DUPLEX_ENABLED);
#elif defined(TARGET_R9SLIMPLUS_RX) /* !TARGET_R9SLIMPLUS_RX */
#define SERIAL_PROTOCOL_RX CrsfRxSerial
HardwareSerial CrsfRxSerial(USART3);
#elif defined(TARGET_RX_FM30_MINI)
#define SERIAL_PROTOCOL_RX SERIAL_PROTOCOL_TX
#else
#define SERIAL_PROTOCOL_RX Serial
#endif

static volatile bool frameAvailable = false;
static volatile bool frameMissed = false;
static enum teamraceOutputInhibitState_e {
    troiPass = 0,               // Allow all packets through, normal operation
    troiDisableAwaitConfirm,    // Have received one packet with another model selected, awaiting confirm to Inhibit
    troiInhibit,                // Inhibit all output
    troiEnableAwaitConfirm,     // Have received one packet with this model selected, awaiting confirm to Pass
} teamraceOutputInhibitState;

SerialIO *serialIO;

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
    frameAvailable = true;
}

void ICACHE_RAM_ATTR crsfRCFrameMissed()
{
    frameMissed = true;
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

/***
 * @brief: Convert the current TeamraceChannel value to the appropriate config value for comparison
*/
static uint8_t teamraceChannelToConfigValue()
{
    // SWITCH3b is 1,2,3,4,5,6,x,Mid
    //             0 1 2 3 4 5    7
    // Config values are Disabled,1,2,3,Mid,4,5,6
    //                      0     1 2 3  4  5 6 7
    uint8_t retVal = CRSF_to_SWITCH3b(ChannelData[config.GetTeamraceChannel()]);
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
static bool confirmFrameAvailable()
{
    if (!frameAvailable)
        return false;
    frameAvailable = false;

    // ModelMatch failure always prevents passing the frame on
    if (!connectionHasModelMatch)
        return false;

    constexpr uint8_t CONFIG_TEAMRACE_POS_OFF = 0;
    if (config.GetTeamracePosition() == CONFIG_TEAMRACE_POS_OFF)
    {
        teamraceOutputInhibitState = troiPass;
        return true;
    }

    // Pass the packet on if in troiPass (of course) or
    // troiDisableAwaitConfirm (keep sending channels until the teamracepos stabilizes)
    bool retVal = teamraceOutputInhibitState < troiInhibit;

    static uint8_t lastTeamracePosition;
    uint8_t newTeamracePosition = teamraceChannelToConfigValue();
    switch (teamraceOutputInhibitState)
    {
        case troiPass:
            // User appears to be switching away from this model, wait for confirm
            if (newTeamracePosition != config.GetTeamracePosition())
                teamraceOutputInhibitState = troiDisableAwaitConfirm;
            break;

        case troiDisableAwaitConfirm:
            // Must receive the same new position twice in a row for state to change
            if (lastTeamracePosition == newTeamracePosition)
            {
                if (newTeamracePosition != config.GetTeamracePosition())
                    teamraceOutputInhibitState = troiInhibit; // disable output
                else
                    teamraceOutputInhibitState = troiPass; // return to normal
            }
            break;

        case troiInhibit:
            // User appears to be switching to this model, wait for confirm
            if (newTeamracePosition == config.GetTeamracePosition())
                teamraceOutputInhibitState = troiEnableAwaitConfirm;
            break;

        case troiEnableAwaitConfirm:
            // Must receive the same new position twice in a row for state to change
            if (lastTeamracePosition == newTeamracePosition)
            {
                if (newTeamracePosition == config.GetTeamracePosition())
                    teamraceOutputInhibitState = troiPass; // return to normal
                else
                    teamraceOutputInhibitState = troiInhibit; // back to disabled
            }
            break;
    }

    lastTeamracePosition = newTeamracePosition;
    // troiPass or troiDisablePending indicate the model is selected still,
    // however returning true if troiDisablePending means this RX could send
    // telemetry and we do not want that
    teamraceHasModelMatch = teamraceOutputInhibitState == troiPass;
    return retVal;
}

static int timeout()
{
    if (connectionState == serialUpdate)
    {
        return DURATION_NEVER;  // stop callbacks when doing serial update
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
    bool missed = frameMissed;
    frameMissed = false;
    interrupts();

    // Verify there is new ChannelData and they should be sent on
    bool sendChannels = confirmFrameAvailable();
    uint32_t duration = serialIO->sendRCFrame(sendChannels, missed, ChannelData);

    // still get telemetry and send link stats if theres no model match
    serialIO->processSerialInput();
    serialIO->sendQueuedData(serialIO->getMaxSerialWriteSize());
    return duration;
}

static void initialize()
{
    uint32_t serialBaud = firmwareOptions.uart_baud;
    bool sbusSerialOutput = false;
    bool sumdSerialOutput = false;

#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
    bool hottTlmSerial = false;
#endif

#if defined(GPIO_PIN_PWM_OUTPUTS)
    // If serial is not already defined, then see if there is serial pin configured in the PWM configuration
    if (GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN)
    {
        for (int i = 0 ; i < GPIO_PIN_PWM_OUTPUTS_COUNT ; i++)
        {
            eServoOutputMode pinMode = (eServoOutputMode)config.GetPwmChannel(i)->val.mode;
            if (pinMode == somSerial)
            {
                pwmSerialDefined = true;
                break;
            }
        }
    }
#endif

    if (OPT_CRSF_RCVR_NO_SERIAL)
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

    if (config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_SBUS || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
    {
        sbusSerialOutput = true;
        serialBaud = 100000;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SUMD)
    {
        sumdSerialOutput = true;
        serialBaud = 115200;
    }
#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
    else if (config.GetSerialProtocol() == PROTOCOL_HOTT_TLM)
    {
        hottTlmSerial = true;
        serialBaud = 19200;
    }
#endif
    bool invert = config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_CRSF || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO;

#ifdef PLATFORM_STM32
#if defined(TARGET_R9SLIMPLUS_RX)
    SERIAL_PROTOCOL_RX.setRx(GPIO_PIN_RCSIGNAL_RX);
    SERIAL_PROTOCOL_RX.begin(serialBaud);

    SERIAL_PROTOCOL_TX.setTx(GPIO_PIN_RCSIGNAL_TX);
#else
#if defined(GPIO_PIN_RCSIGNAL_RX_SBUS) && defined(GPIO_PIN_RCSIGNAL_TX_SBUS)
    if (invert)
    {
        SERIAL_PROTOCOL_TX.setTx(GPIO_PIN_RCSIGNAL_TX_SBUS);
        SERIAL_PROTOCOL_TX.setRx(GPIO_PIN_RCSIGNAL_RX_SBUS);
    }
    else
#endif
    {
        SERIAL_PROTOCOL_TX.setTx(GPIO_PIN_RCSIGNAL_TX);
        SERIAL_PROTOCOL_TX.setRx(GPIO_PIN_RCSIGNAL_RX);
    }
#endif /* TARGET_R9SLIMPLUS_RX */
#if defined(TARGET_RX_GHOST_ATTO_V1)
    // USART1 is used for RX (half duplex)
    SERIAL_PROTOCOL_RX.setHalfDuplex();
    SERIAL_PROTOCOL_RX.setTx(GPIO_PIN_RCSIGNAL_RX);
    SERIAL_PROTOCOL_RX.begin(serialBaud);
    SERIAL_PROTOCOL_RX.enableHalfDuplexRx();

    // USART2 is used for TX (half duplex)
    // Note: these must be set before begin()
    SERIAL_PROTOCOL_TX.setHalfDuplex();
    SERIAL_PROTOCOL_TX.setRx((PinName)NC);
    SERIAL_PROTOCOL_TX.setTx(GPIO_PIN_RCSIGNAL_TX);
#endif /* TARGET_RX_GHOST_ATTO_V1 */
    SERIAL_PROTOCOL_TX.begin(serialBaud, sbusSerialOutput ? SERIAL_8E2 : SERIAL_8N1);
#endif /* PLATFORM_STM32 */
#if defined(TARGET_RX_GHOST_ATTO_V1) || defined(TARGET_RX_FM30_MINI)
    if (invert)
    {
        LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_2, LL_GPIO_PULL_DOWN);
        USART2->CR1 &= ~USART_CR1_UE;
        USART2->CR2 |= USART_CR2_TXINV;
        USART2->CR1 |= USART_CR1_UE;
    }
#endif

#if defined(TARGET_RX_FM30_MINI) || defined(TARGET_DIY_900_RX_STM32)
    Serial.setRx(GPIO_PIN_DEBUG_RX);
    Serial.setTx(GPIO_PIN_DEBUG_TX);
    Serial.begin(serialBaud); // Same baud as CRSF for simplicity
#endif

#if defined(PLATFORM_ESP8266)
    SerialConfig serialConfig = SERIAL_8N1;

    if(sbusSerialOutput)
    {
        serialConfig = SERIAL_8E2;
    }
    else if(hottTlmSerial)
    {
        serialConfig = SERIAL_8N2;
    }

    SerialMode mode = (sbusSerialOutput || sumdSerialOutput)  ? SERIAL_TX_ONLY : SERIAL_FULL;
    Serial.begin(serialBaud, serialConfig, mode, -1, invert);
#elif defined(PLATFORM_ESP32)
    uint32_t serialConfig = SERIAL_8N1;

    if(sbusSerialOutput)
    {
        serialConfig = SERIAL_8E2;
    }
    else if(hottTlmSerial)
    {
        serialConfig = SERIAL_8N2;
    }

    Serial.begin(serialBaud, serialConfig, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, invert);
#endif

    if (firmwareOptions.is_airport)
    {
        serialIO = new SerialAirPort(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (sbusSerialOutput)
    {
        serialIO = new SerialSBUS(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (sumdSerialOutput)
    {
        serialIO = new SerialSUMD(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
    else if (hottTlmSerial)
    {
        serialIO = new SerialHoTT_TLM(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
#endif
    else
    {
        serialIO = new SerialCRSF(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
#if defined(PLATFORM_ESP32_S3)
    USBSerial.begin(460800);
    SerialLogger = &USBSerial;
#else
    SerialLogger = &Serial;
#endif
}

static void serialShutdown()
{
    SerialLogger = new NullStream();
#ifdef PLATFORM_STM32
#if defined(TARGET_R9SLIMPLUS_RX) || defined(TARGET_RX_GHOST_ATTO_V1)
    SERIAL_PROTOCOL_RX.end();
#endif
    SERIAL_PROTOCOL_TX.end();
#else
    Serial.end();
#endif
    delete serialIO;
}

void reconfigureSerial()
{
    serialShutdown();
    initialize();
}

device_t Serial_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout
};

#endif