#pragma once

#include "SerialIO.h"
#include "FIFO.h"

#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS 1
#endif
#include "common/mavlink.h"

#define MAV_INPUT_BUF_LEN       1024
#define MAV_OUTPUT_BUF_LEN      512
#define MAV_PAYLOAD_SIZE_MAX    60

// Variables / constants
extern FIFO<MAV_INPUT_BUF_LEN> mavlinkInputBuffer;
extern FIFO<MAV_OUTPUT_BUF_LEN> mavlinkOutputBuffer;

class SerialMavlink final : public SerialIO {
public:
    explicit SerialMavlink(Stream &out, Stream &in);
    ~SerialMavlink() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    void forwardMessage(const uint8_t *data);
    bool GetNextPayload(uint8_t *nextPayloadSize, uint8_t *payloadData);

    void event() override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;

    uint8_t this_system_id;
    const uint8_t this_component_id;

    uint8_t target_system_id;
    const uint8_t target_component_id;

    uint32_t lastSentFlowCtrl = 0;

    // Variables / constants for Mavlink //
    FIFO<MAV_INPUT_BUF_LEN> mavlinkInputBuffer;
    FIFO<MAV_OUTPUT_BUF_LEN> mavlinkOutputBuffer;

    uint8_t currentOutput[MAVLINK_MAX_PACKET_LEN];
    uint16_t currentOutputSize = 0;
    uint16_t currentOutputOffset = 0;
    bool currentOutputIsRadioStatus = false;

    uint8_t pendingRc[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
    uint16_t pendingRcSize = 0;
};
