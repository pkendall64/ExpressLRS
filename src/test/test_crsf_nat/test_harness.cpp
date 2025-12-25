#include "test_harness.h"
#include <cstring>

void LinkConnector::setPeer(CRSFRouter &router, LinkConnector &connector)
{
    peerRouter = &router;
    peerConnector = &connector;
}

void LinkConnector::enableNat(const bool enable)
{
    natEnabled = enable;
}

void LinkConnector::advertise(crsf_addr_e addr)
{
    addDevice(addr);
}

void LinkConnector::forwardMessage(const crsf_header_t *message)
{
    forwardMessageCalled = true;
    if (message) {
        if (message->type < CRSF_FRAMETYPE_DEVICE_PING)
            memcpy(&lastForwardedMessage, message, sizeof(crsf_header_t));
        else
            memcpy(&lastForwardedMessage, message, sizeof(crsf_ext_header_t));
        if (message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
            forwardedMessages.push_back(lastForwardedMessage);
        }
    }
    if (peerRouter && peerConnector) {
        peerRouter->processMessage(peerConnector, message);
    }
}

bool LinkConnector::shouldNatDevice(crsf_addr_e device_id)
{
    if (!natEnabled) return false;
    return device_id == CRSF_ADDRESS_CRSF_RECEIVER || device_id == CRSF_ADDRESS_CRSF_TRANSMITTER;
}

void LinkConnector::reset()
{
    forwardMessageCalled = false;
    memset(&lastForwardedMessage, 0, sizeof(lastForwardedMessage));
    forwardedMessages.clear();
}

void TestEndpoint::handleMessage(const crsf_header_t *message)
{
    if (!message) return;

    // For tests: when we receive a PING (extended frame), trigger a parameter update request
    if (message->type >= CRSF_FRAMETYPE_DEVICE_PING) {
        const auto *ext = reinterpret_cast<const crsf_ext_header_t *>(message);
        parameterUpdateReq(ext->orig_addr, false, CRSF_FRAMETYPE_DEVICE_PING, 0, 0);
    }
}

void create_test_message(crsf_ext_header_t *msg, crsf_frame_type_e type, crsf_addr_e dest, crsf_addr_e origin)
{
    memset(msg, 0, sizeof(crsf_ext_header_t));
    msg->device_addr = dest;
    msg->frame_size = 4; // minimal ext header
    msg->type = type;
    msg->dest_addr = dest;
    msg->orig_addr = origin;
}
