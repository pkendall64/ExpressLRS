#if defined(TARGET_RX)

#include "SerialMavlink.h"
#include "OTA.h"
#include "common.h"
#include "config.h"
#include "device.h"

#define MAVLINK_RC_PACKET_INTERVAL 10


#define MAV_FTP_OPCODE_OPENFILERO 4

SerialMavlink::SerialMavlink(Stream &out, Stream &in):
    SerialIO(&out, &in),

    //system ID of the device component sending command to FC, can be set using lua options, 0 is the default value for initialized storage, treat it as 255 which is commonly used as GCS SysID
    this_system_id(config.GetSourceSysId() ? config.GetSourceSysId() : 255),
    //use telemetry radio compId as we are providing radio status messages and pass telemetry
    this_component_id(MAV_COMPONENT::MAV_COMP_ID_TELEMETRY_RADIO),

    // system ID of vehicle we want to control must be the same as target vehicle, can be set using lua options, 0 is the default value for initialized storage, treat it as 1 which is commonly used as UAV SysID in 1:1 networks
    target_system_id(config.GetTargetSysId() ? config.GetTargetSysId() : 1),
    // Send to all components as we may have ex. gimbal that listens to RC instead of using Autopilot driver
    target_component_id(MAV_COMPONENT::MAV_COMP_ID_ALL)
{
}

uint32_t SerialMavlink::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    if (!frameAvailable) {
        return DURATION_IMMEDIATELY;
    }

    const mavlink_rc_channels_override_t rc_override {
        .chan1_raw = CRSF_to_US(channelData[0]),
        .chan2_raw = CRSF_to_US(channelData[1]),
        .chan3_raw = CRSF_to_US(channelData[2]),
        .chan4_raw = CRSF_to_US(channelData[3]),
        .chan5_raw = CRSF_to_US(channelData[4]),
        .chan6_raw = CRSF_to_US(channelData[5]),
        .chan7_raw = CRSF_to_US(channelData[6]),
        .chan8_raw = CRSF_to_US(channelData[7]),
        .target_system = target_system_id,
        .target_component = target_component_id,
        .chan9_raw = CRSF_to_US(channelData[8]),
        .chan10_raw = CRSF_to_US(channelData[9]),
        .chan11_raw = CRSF_to_US(channelData[10]),
        .chan12_raw = CRSF_to_US(channelData[11]),
        .chan13_raw = CRSF_to_US(channelData[12]),
        .chan14_raw = CRSF_to_US(channelData[13]),
        .chan15_raw = CRSF_to_US(channelData[14]),
        .chan16_raw = CRSF_to_US(channelData[15]),
    };

    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_encode(this_system_id, this_component_id, &msg, &rc_override);
    pendingRcSize = mavlink_msg_to_send_buffer(pendingRc, &msg);

    return MAVLINK_RC_PACKET_INTERVAL;
}

int SerialMavlink::getMaxSerialReadSize()
{
    return MAV_INPUT_BUF_LEN - mavlinkInputBuffer.size();
}

void SerialMavlink::processBytes(uint8_t *bytes, u_int16_t size)
{
    if (connectionState == connected)
    {
        mavlinkInputBuffer.atomicPushBytes(bytes, size);
    }
}

void SerialMavlink::sendQueuedData(uint32_t maxBytesToSend)
{
    uint32_t bytesWritten = 0;
    const uint32_t now = millis();

    while (bytesWritten < maxBytesToSend)
    {
        if (currentOutputOffset < currentOutputSize)
        {
            const uint16_t remaining = currentOutputSize - currentOutputOffset;
            const uint16_t len = std::min<uint32_t>(remaining, maxBytesToSend - bytesWritten);
            const size_t written = _outputPort->write(currentOutput + currentOutputOffset, len);
            currentOutputOffset += written;
            bytesWritten += written;
            if (written != len)
            {
                return;
            }
            if (currentOutputOffset == currentOutputSize)
            {
                if (currentOutputIsRadioStatus)
                {
                    lastSentFlowCtrl = millis();
                }
                currentOutputSize = 0;
                currentOutputOffset = 0;
                currentOutputIsRadioStatus = false;
            }
            continue;
        }

        if (pendingRcSize != 0)
        {
            memcpy(currentOutput, pendingRc, pendingRcSize);
            currentOutputSize = pendingRcSize;
            currentOutputIsRadioStatus = false;
            pendingRcSize = 0;
            continue;
        }

        if ((now - lastSentFlowCtrl) > 10)
        {
            uint8_t percentage_remaining = ((MAV_INPUT_BUF_LEN - mavlinkInputBuffer.size()) * 100) / MAV_INPUT_BUF_LEN;
            const mavlink_radio_status_t radio_status {
                .rxerrors = 0,
                .fixed = 0,
                .rssi = (uint8_t)((float)linkStats.uplink_Link_quality * 2.55),
                .remrssi = linkStats.uplink_RSSI_1,
                .txbuf = percentage_remaining,
                .noise = (uint8_t)linkStats.uplink_SNR,
                .remnoise = 0,
            };
            mavlink_message_t msg;
            mavlink_msg_radio_status_encode(this_system_id, this_component_id, &msg, &radio_status);
            currentOutputSize = mavlink_msg_to_send_buffer(currentOutput, &msg);
            currentOutputOffset = 0;
            currentOutputIsRadioStatus = true;
            continue;
        }

        mavlinkOutputBuffer.lock();
        if (mavlinkOutputBuffer.size() < 2)
        {
            mavlinkOutputBuffer.unlock();
            return;
        }
        const uint16_t len = mavlinkOutputBuffer[0] | (mavlinkOutputBuffer[1] << 8);
        if (len > sizeof(currentOutput) || mavlinkOutputBuffer.size() < len + 2)
        {
            mavlinkOutputBuffer.flush();
            mavlinkOutputBuffer.unlock();
            return;
        }
        mavlinkOutputBuffer.pop();
        mavlinkOutputBuffer.pop();
        mavlinkOutputBuffer.popBytes(currentOutput, len);
        mavlinkOutputBuffer.unlock();
        currentOutputSize = len;
        currentOutputIsRadioStatus = false;
        currentOutputOffset = 0;
    }
}

void SerialMavlink::event()
{
    this_system_id = config.GetSourceSysId() ? config.GetSourceSysId() : 255;
    target_system_id = config.GetTargetSysId() ? config.GetTargetSysId() : 1;
}

void SerialMavlink::forwardMessage(const uint8_t *data)
{
    for (uint8_t i = 0; i < data[1]; ++i)
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_frame_char(MAVLINK_COMM_0, data[i + 2], &msg, &status) == MAVLINK_FRAMING_OK)
        {
            uint8_t frame[MAVLINK_MAX_PACKET_LEN];
            const uint16_t len = mavlink_msg_to_send_buffer(frame, &msg);
            mavlinkOutputBuffer.lock();
            if (mavlinkOutputBuffer.available(len + 2))
            {
                mavlinkOutputBuffer.push(len & 0xff);
                mavlinkOutputBuffer.push(len >> 8);
                mavlinkOutputBuffer.pushBytes(frame, len);
            }
            mavlinkOutputBuffer.unlock();
        }
    }
}

bool SerialMavlink::GetNextPayload(uint8_t* nextPayloadSize, uint8_t *payloadData)
{
    if (mavlinkInputBuffer.size() == 0)
    {
        return false;
    }
    const uint16_t count = std::min(mavlinkInputBuffer.size(), (uint16_t)CRSF_PAYLOAD_SIZE_MAX); // Constrain to CRSF max payload size to match SS
    payloadData[0] = CRSF_ADDRESS_USB; // device_addr - used on TX to differentiate between std tlm and mavlink
    payloadData[1] = count;
    // The following 'n' bytes are just raw mavlink
    mavlinkInputBuffer.popBytes(payloadData + CRSF_FRAME_NOT_COUNTED_BYTES, count);
    *nextPayloadSize = count + CRSF_FRAME_NOT_COUNTED_BYTES;
    return true;
}

#endif // defined(TARGET_RX)
