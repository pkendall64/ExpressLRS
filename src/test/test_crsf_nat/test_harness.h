// Test harness for CRSF NAT tests (test-only)
#pragma once

#include "CRSFRouter.h"
#include <vector>

// LinkConnector bridges two routers' connectors, optionally enabling NAT table entries
class LinkConnector final : public CRSFConnector {
public:
    bool forwardMessageCalled = false;
    crsf_ext_header_t lastForwardedMessage{};
    std::vector<crsf_ext_header_t> forwardedMessages; // records extended frames

    CRSFRouter *peerRouter = nullptr;
    LinkConnector *peerConnector = nullptr;
    bool natEnabled = false;

    void setPeer(CRSFRouter &router, LinkConnector &connector);
    void enableNat(bool enable);
    void advertise(crsf_addr_e addr);

    void forwardMessage(const crsf_header_t *message) override;
    bool shouldNatDevice(crsf_addr_e device_id) override;
    void reset();
};

// Endpoint that delegates to CRSFEndpoint base to keep auto-reply behavior
class TestEndpoint final : public CRSFEndpoint {
public:
    TestEndpoint(const CRSF &crsf, const crsf_addr_e device_id) : CRSFEndpoint(crsf, device_id) {}
    void handleMessage(const crsf_header_t *message) override;
};

// Helper to build a minimal extended frame for routing tests
void create_test_message(crsf_ext_header_t *msg, crsf_frame_type_e type, crsf_addr_e dest, crsf_addr_e origin);
