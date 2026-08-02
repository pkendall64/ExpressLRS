#include "Backpack.h"

#include "targets.h"

#include "CRSFHandset.h"
#include "CRSFRouter.h"
#include "config.h"
#include "devVTX.h"
#include "logging.h"
#include "msp.h"
#include "msptypes.h"
#include "rxtx_intf.h"

// value in ptrChannelData that means "don't replace value in ChannelData"
static constexpr uint16_t HT_DO_NOT_UPDATE =    0xffffU;
// EdgeTX doesn't support not updating a value, so 0 might be a safer value?
// it always takes (11-bits) - CRSF_CHANNEL_VALUE_MID * 5/8
static constexpr uint32_t HT_EDGETX_NO_VALUE =  0U;
// Stop overriding channels with PTR data if older than this
static constexpr uint32_t HT_STALE_TIMEOUT_MS = 1000U;

#if defined(PLATFORM_ESP32)

namespace
{
MSP msp;
bool lastRecordingState = false;
uint16_t ptrChannelData[CRSF_NUM_CHANNELS];
bool headTrackingEnabled = false;
uint32_t lastPTRValidTimeMs;
char backpackVersion[32] = "";
bool TxBackpackWiFiReadyToSend = false;
bool VRxBackpackWiFiReadyToSend = false;
bool BackpackTelemReadyToSend = false;

void BackpackWiFiToMSPOut(const uint16_t command)
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = command;
    packet.addByte(0);

    MSP::sendPacket(&packet, BackpackOrLogStrm); // send to tx-backpack as MSP
}

void BackpackHTFlagToMSPOut(const uint8_t arg)
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_SET_HEAD_TRACKING;
    packet.addByte(arg);

    MSP::sendPacket(&packet, BackpackOrLogStrm); // send to tx-backpack as MSP
}

uint8_t GetDvrDelaySeconds(const uint8_t index)
{
    constexpr uint8_t delays[] = {0, 5, 15, 30, 45, 60, 120};
    return delays[index >= sizeof(delays) ? 0 : index];
}

void BackpackDvrRecordingStateMSPOut(bool recordingState)
{
    uint16_t delay = 0;

    if (recordingState)
    {
        delay = GetDvrDelaySeconds(config.GetDvrStartDelay());
    }
    else
    {
        delay = GetDvrDelaySeconds(config.GetDvrStopDelay());
    }

    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_SET_RECORDING_STATE;
    packet.addByte(recordingState);
    packet.addByte(delay & 0xFF); // delay byte 1
    packet.addByte(delay >> 8);   // delay byte 2

    MSP::sendPacket(&packet, BackpackOrLogStrm); // send to tx-backpack as MSP
}

void BackpackBinding()
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BIND;
    for (const uint8_t b : UID)
    {
        packet.addByte(b);
    }

    MSP::sendPacket(&packet, BackpackOrLogStrm); // send to tx-backpack as MSP
}

/***
 * @brief:  Read 16-bit CRSF value channels from the packet payload and store them for head-tracking/trainer.
 *          Process as many values as there are payload bytes available
 *          A value of HT_DO_NOT_UPDATE will not overwrite that channel
 */
void processPanTiltRollPacket(const uint32_t now, const mspPacket_t *packet)
{
    unsigned chStart = (config.GetPTRStartChannel() == HT_START_EDGETX) ? 0 : (config.GetPTRStartChannel() - HT_START_AUX1 + AUX1);
    unsigned payloadPos = 0;
    // Any time a PTR packet is received, all channels are returned to HT_DO_NOT_UPDATE unless they are in this packet
    for (unsigned ch=0; ch<CRSF_NUM_CHANNELS; ++ch)
    {
        uint16_t chVal;
        if (ch >= chStart && payloadPos < (packet->payloadSize - 1))
        {
            chVal = packet->payload[payloadPos] | (packet->payload[payloadPos+1] << 8);
            payloadPos += 2;
        }
        else
        {
            chVal = HT_DO_NOT_UPDATE;
        }
        ptrChannelData[ch] = chVal;
    }

    lastPTRValidTimeMs = now;
}

void headtrackPublishChannelsToEdgeTX()
{
    static uint32_t lastPTRSentMs = 0;
    if (lastPTRSentMs == lastPTRValidTimeMs)
        return;
    lastPTRSentMs = lastPTRValidTimeMs;

    CRSF_MK_FRAME_T(crsf_channels_t) rcPacket; // not zeroed, entire packet will be filled
    // Pack the ptrChannelData into 11 bits for the crsf_channels_t packet
    constexpr unsigned dstBits = 11;
    uint8_t *dst = reinterpret_cast<uint8_t *>(&rcPacket.p);
    uint32_t accumulator = 0;
    uint32_t bitCnt = 0;
    for (unsigned ch=0; ch<CRSF_NUM_CHANNELS; ++ch)
    {
        uint32_t val = ptrChannelData[ch] == HT_DO_NOT_UPDATE ? HT_EDGETX_NO_VALUE : ptrChannelData[ch];
        accumulator |= val << bitCnt;
        bitCnt += dstBits;
        while (bitCnt >= 8)
        {

            *dst++ = accumulator;
            accumulator >>= 8;
            bitCnt -= 8;
        }
    }
    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&rcPacket, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, sizeof(rcPacket) - 2);
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &rcPacket.h);
}

void headtrackOverrideChannels(uint32_t channels[], size_t channelCount)
{
    if (millis() - lastPTRValidTimeMs > HT_STALE_TIMEOUT_MS)
        return;

    channelCount = std::min((size_t)CRSF_NUM_CHANNELS, channelCount);
    for (unsigned ch=0; ch<channelCount; ++ch)
    {
        const auto val = ptrChannelData[ch];
        if (val != HT_DO_NOT_UPDATE)
            channels[ch] = val;
    }
}

void BackpackPollAuxStates()
{
    // HeadTracking Enable
    bool enable = backpackVersion[0] != 0;
    if (enable)
    {
        switch (config.GetPTREnableChannel())
        {
        case HT_OFF:
            enable = false; break;
        case HT_ON:
            enable = true; break;
        default:
            enable = CRSF_to_BIT(ChannelData[config.GetPTREnableChannel() / 2 + 3]);
            if (config.GetPTREnableChannel() % 2)
                enable = !enable;
        }
    }
    if (enable != headTrackingEnabled)
    {
        headTrackingEnabled = enable;
        BackpackHTFlagToMSPOut(headTrackingEnabled);

        auto rcchannelsoverride_cb = (enable && config.GetPTRStartChannel() != HT_START_EDGETX) ? &headtrackOverrideChannels : nullptr;
        handset->setRcChannelsOverrideCallback(rcchannelsoverride_cb);

        auto rcdata_cb = (enable && config.GetPTRStartChannel() == HT_START_EDGETX) ? &headtrackPublishChannelsToEdgeTX : nullptr;
        handset->setRCDataCallback(rcdata_cb);
    }

    // DVR recording enable
    if (config.GetDvrAux() != 0)
    {
        // DVR AUX control is on
        const uint8_t auxNumber = (config.GetDvrAux() - 1) / 2 + AUX1;
        const uint8_t auxInverted = (config.GetDvrAux() + 1) % 2;

        const bool recordingState = CRSF_to_BIT(ChannelData[auxNumber]) ^ auxInverted;
        if (recordingState != lastRecordingState)
        {
            // Channel state has changed since we last checked, so schedule a MSP send
            lastRecordingState = recordingState;
            BackpackDvrRecordingStateMSPOut(recordingState);
        }
    }
}

void sendConfigToBackpack()
{
    // Send any config values to the tx-backpack, as one key/value pair per MSP msg
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_CONFIG;
    packet.addByte(MSP_ELRS_BACKPACK_CONFIG_TLM_MODE); // Backpack tlm mode
    packet.addByte(config.GetBackpackTlmMode());
    MSP::sendPacket(&packet, BackpackOrLogStrm); // send to tx-backpack as MSP
}

void ProcessMSPPacket(uint32_t now, mspPacket_t *packet)
{
    // Inspect packet for ELRS specific opcodes
#if 0
    // FIXME: we should just remove the power calibration thing, it's never been used and is deprecated since hardware.json
    if (packet->function == MSP_ELRS_FUNC)
    {
        uint8_t opcode = packet->readByte();

        CHECK_PACKET_PARSING();

        switch (opcode)
        {
        case MSP_ELRS_POWER_CALI_GET:
            OnPowerGetCalibration(packet);
            break;
        case MSP_ELRS_POWER_CALI_SET:
            OnPowerSetCalibration(packet);
            break;
        default:
            break;
        }
    }
#endif
    if (packet->function == MSP_SET_VTX_CONFIG)
    {
        if (packet->payload[0] < 48) // Standard 48 channel VTx table size e.g. A, B, E, F, R, L
        {
            config.SetVtxBand(packet->payload[0] / 8 + 1);
            config.SetVtxChannel(packet->payload[0] % 8);
        } else
        {
            return; // Packets containing frequency in MHz are not yet supported.
        }

        VtxTriggerSend();
    }
    else if (packet->function == MSP_ELRS_BACKPACK_SET_PTR)
    {
        processPanTiltRollPacket(now, packet);
    }
    if (packet->function == MSP_ELRS_GET_BACKPACK_VERSION)
    {
        memset(backpackVersion, 0, sizeof(backpackVersion));
        memcpy(backpackVersion, packet->payload, min((size_t)packet->payloadSize, sizeof(backpackVersion)-1));
    }
}
}

const char *getBackpackVersion()
{
    return backpackVersion;
}

void sendBackpackCommand(backpackCommand_e command)
{
    switch (command)
    {
    case ENABLE_TXBP_WIFI:
        TxBackpackWiFiReadyToSend = true;
        break;
    case ENABLE_VRX_WIFI:
        VRxBackpackWiFiReadyToSend = true;
        break;
    case SEND_TELEMETRY_CONFIG:
        BackpackTelemReadyToSend = true;
        break;
    default:
        break;
    }
}

bool getBackpackCommandState(backpackCommand_e command)
{
    switch (command)
    {
    case ENABLE_TXBP_WIFI:
        return TxBackpackWiFiReadyToSend;
    case ENABLE_VRX_WIFI:
        return VRxBackpackWiFiReadyToSend;
    case SEND_TELEMETRY_CONFIG:
        return BackpackTelemReadyToSend;
    default:
        return false;
    }
}

void resetBackpackChannelData()
{
    // Set all channels of PTR data to "do not override" (0xffff)
    memset(ptrChannelData, 0xff, sizeof(ptrChannelData));
}

void ParseMSPData(uint8_t *buf, uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i)
    {
        if (msp.processReceivedByte(buf[i]))
        {
            ProcessMSPPacket(millis(), msp.getReceivedPacket());
            msp.markPacketReceived();
        }
    }
}

void ProcessPendingCommands()
{
    static uint8_t versionRequestTries = 0;
    static uint32_t lastVersionTryTime = 0;

    if (versionRequestTries < 10 && strlen(backpackVersion) == 0 && (lastVersionTryTime == 0 || millis() - lastVersionTryTime > 1000))
    {
        lastVersionTryTime = millis();
        versionRequestTries++;
        mspPacket_t out;
        out.reset();
        out.makeCommand();
        out.function = MSP_ELRS_GET_BACKPACK_VERSION;
        MSP::sendPacket(&out, BackpackOrLogStrm);
        DBGLN("Sending get backpack version command");
    }

    if (connectionState < MODE_STATES && !config.GetBackpackDisable())
    {
        if (TxBackpackWiFiReadyToSend)
        {
            TxBackpackWiFiReadyToSend = false;
            BackpackWiFiToMSPOut(MSP_ELRS_SET_TX_BACKPACK_WIFI_MODE);
        }

        if (VRxBackpackWiFiReadyToSend)
        {
            VRxBackpackWiFiReadyToSend = false;
            BackpackWiFiToMSPOut(MSP_ELRS_SET_VRX_BACKPACK_WIFI_MODE);
        }

        if (BackpackTelemReadyToSend)
        {
            BackpackTelemReadyToSend = false;
            sendConfigToBackpack();
        }

        BackpackPollAuxStates();
    }
}

void ProcessEvents(bool disabled)
{
    if (InBindingMode)
    {
        BackpackBinding();
    }

    if (disabled && headTrackingEnabled)
    {
        // Disconnect any handlers that might be active. This is done blindly and will disconnect callbacks from other
        // devices if they are assigned in their own event() handlers that fire before this!
        // Note there's no MSP_ELRS_BACKPACK_SET_PTR sent either, as the backpack is disabled above
        headTrackingEnabled = false;
        handset->setRcChannelsOverrideCallback(nullptr);
        handset->setRCDataCallback(nullptr);
    }
}

#endif
