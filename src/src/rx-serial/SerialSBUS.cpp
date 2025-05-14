#include "SerialSBUS.h"
#include "config.h"
#include "crsf_protocol.h"
#include "device.h"

#define SBUS_FLAG_SIGNAL_LOSS (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE (1 << 3)

constexpr auto UNCONNECTED_CALLBACK_INTERVAL_MS = 10;
constexpr auto SBUS_CALLBACK_INTERVAL_MS = 9;

int32_t SerialSBUS::sendRCFrame(const bool frameAvailable, const bool frameMissed, uint32_t *channelData)
{
    static auto sendPackets = false;
    const bool effectivelyFailsafed = failsafe || (!connectionHasModelMatch) || (!teamraceHasModelMatch);
    if ((effectivelyFailsafed && config.GetFailsafeMode() == FAILSAFE_NO_PULSES) || (!sendPackets && connectionState != connected))
    {
        return UNCONNECTED_CALLBACK_INTERVAL_MS;
    }
    sendPackets = true;

    if ((!frameAvailable && !frameMissed && !effectivelyFailsafed) || _stream->availableForWrite() < 25)
    {
        return DURATION_IMMEDIATELY;
    }

    // TODO: if failsafeMode == FAILSAFE_SET_POSITION then we use the set positions rather than the last values
    crsf_channels_s PackedRCdataOut{};

    if (isDjiRsPro)
    {
        PackedRCdataOut.ch0 = fmap(channelData[0], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch1 = fmap(channelData[1], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch2 = fmap(channelData[2], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch3 = fmap(channelData[3], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch4 = fmap(channelData[5], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Record start/stop and photo
        PackedRCdataOut.ch5 = fmap(channelData[6], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Mode
        PackedRCdataOut.ch6 = fmap(channelData[7], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 176, 848);  // Recenter and Selfie
        PackedRCdataOut.ch7 = fmap(channelData[8], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch8 = fmap(channelData[9], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch9 = fmap(channelData[10], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch10 = fmap(channelData[11], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch11 = fmap(channelData[12], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch12 = fmap(channelData[13], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch13 = fmap(channelData[14], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch14 = fmap(channelData[15], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch15 = channelData[4] < CRSF_CHANNEL_VALUE_MID ? 352 : 1696;
    }
    else
    {
        PackedRCdataOut.ch0 = channelData[0];
        PackedRCdataOut.ch1 = channelData[1];
        PackedRCdataOut.ch2 = channelData[2];
        PackedRCdataOut.ch3 = channelData[3];
        PackedRCdataOut.ch4 = channelData[4];
        PackedRCdataOut.ch5 = channelData[5];
        PackedRCdataOut.ch6 = channelData[6];
        PackedRCdataOut.ch7 = channelData[7];
        PackedRCdataOut.ch8 = channelData[8];
        PackedRCdataOut.ch9 = channelData[9];
        PackedRCdataOut.ch10 = channelData[10];
        PackedRCdataOut.ch11 = channelData[11];
        PackedRCdataOut.ch12 = channelData[12];
        PackedRCdataOut.ch13 = channelData[13];
        PackedRCdataOut.ch14 = channelData[14];
        PackedRCdataOut.ch15 = channelData[15];
    }

    uint8_t extraData = 0;
    extraData |= effectivelyFailsafed ? SBUS_FLAG_FAILSAFE_ACTIVE : 0;
    extraData |= frameMissed ? SBUS_FLAG_SIGNAL_LOSS : 0;

    _stream->write(0x0F); // HEADER
    _stream->write((byte *)&PackedRCdataOut, sizeof(PackedRCdataOut));
    _stream->write(extraData);                  // ch 17, 18, lost packet, failsafe
    _stream->write(static_cast<uint8_t>(0x00)); // FOOTER
    return SBUS_CALLBACK_INTERVAL_MS;
}
