#include <cstring>

#include "unity.h"
#include "CRSFRouter.h"
#include "crsf_protocol.h"
#include "test_harness.h"

// Local helper to craft a minimal extended frame for tests
static void make_ext_frame(crsf_ext_header_t *msg,
                           crsf_frame_type_e type,
                           crsf_addr_e dest,
                           crsf_addr_e orig)
{
    memset(msg, 0, sizeof(crsf_ext_header_t));
    msg->device_addr = CRSF_ADDRESS_BROADCAST;
    // Minimal extended header with no payload: type + dest + orig + crc
    msg->frame_size = CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // conforms to router’s CRC calc usage
    msg->type = type;
    msg->dest_addr = dest;
    msg->orig_addr = orig;
}

void test_natMessage_rewrites_origin_and_updates_crc()
{
    LinkConnector connector;

    // Seed a test NAT mapping original -> alias
    constexpr crsf_addr_e original = CRSF_ADDRESS_CRSF_TRANSMITTER;
    constexpr crsf_addr_e alias = CRSF_ADDRESS_NAT_START;
    connector.test_addNatMapping(original, alias);
    connector.enableNat(true);

    // Build input extended frame where origin matches the mapping
    crsf_ext_header_t in{};
    make_ext_frame(&in, CRSF_FRAMETYPE_DEVICE_PING, CRSF_ADDRESS_CRSF_RECEIVER, original);

    const crsf_ext_header_t *out = connector.natMessage(&in);

    // Should use internal NAT buffer (not the same pointer)
    TEST_ASSERT_NOT_EQUAL(&in, out);
    TEST_ASSERT_EQUAL_UINT8(alias, out->orig_addr);

    // Recompute CRC and compare
    const uint8_t *outBytes = reinterpret_cast<const uint8_t *>(out);
    const uint8_t computed = CRSF::CRC.calc(outBytes + CRSF_FRAME_NOT_COUNTED_BYTES,
                                                  out->frame_size - CRSF_TELEMETRY_CRC_LENGTH);
    const uint8_t outCrc = outBytes[out->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES - CRSF_TELEMETRY_CRC_LENGTH];
    TEST_ASSERT_EQUAL_UINT8(computed, outCrc);

    // Input is not modified in place
    TEST_ASSERT_EQUAL_UINT8(original, in.orig_addr);
}

void test_deNatMessage_rewrites_origin_back_to_original_and_updates_crc()
{
    LinkConnector connector;

    constexpr crsf_addr_e original = CRSF_ADDRESS_CRSF_TRANSMITTER;
    constexpr crsf_addr_e alias = CRSF_ADDRESS_NAT_START;
    connector.test_addNatMapping(original, alias);

    // For deNAT, mapping is looked up by dest_addr == alias; orig will be rewritten to original
    crsf_ext_header_t in{};
    make_ext_frame(&in, CRSF_FRAMETYPE_DEVICE_PING, alias, CRSF_ADDRESS_CRSF_RECEIVER);

    const crsf_ext_header_t *out = connector.deNatMessage(&in);

    TEST_ASSERT_NOT_EQUAL(&in, out);
    TEST_ASSERT_EQUAL_UINT8(original, out->dest_addr);

    const uint8_t *outBytes = reinterpret_cast<const uint8_t *>(out);
    const uint8_t computed = CRSF::CRC.calc(outBytes + CRSF_FRAME_NOT_COUNTED_BYTES,
                                                  out->frame_size - CRSF_TELEMETRY_CRC_LENGTH);
    const uint8_t outCrc = outBytes[out->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES - CRSF_TELEMETRY_CRC_LENGTH];
    TEST_ASSERT_EQUAL_UINT8(computed, outCrc);
}

// --- 4-router NAT chaining test moved from endpoint suite ---
void test_NAT_Chained_TX_RXR_TXR_RX()
{
    // Routers
    CRSFRouter R1; // TX
    CRSFRouter R2; // RXR (relay)
    CRSFRouter R3; // TXR (relay)
    CRSFRouter R4; // RX

    // Connectors
    LinkConnector R1C1;
    LinkConnector R1C2;
    LinkConnector R2C1;
    LinkConnector R2C2;
    LinkConnector R3C1;
    LinkConnector R3C2;
    LinkConnector R4C1;
    LinkConnector R4C2;

    // Register connectors
    R1.addConnector(&R1C1); R1.addConnector(&R1C2);
    R2.addConnector(&R2C1); R2.addConnector(&R2C2);
    R3.addConnector(&R3C1); R3.addConnector(&R3C2);
    R4.addConnector(&R4C1); R4.addConnector(&R4C2);

    // Edge knowledge
    R1C1.advertise(CRSF_ADDRESS_RADIO_TRANSMITTER);
    R3C1.advertise(CRSF_ADDRESS_RADIO_TRANSMITTER);
    R2C2.advertise(CRSF_ADDRESS_FLIGHT_CONTROLLER);
    R4C2.advertise(CRSF_ADDRESS_FLIGHT_CONTROLLER);

    // NAT roles
    R2C2.enableNat(true); // RXR connector 2
    R3C1.enableNat(true); // TXR connector 1

    // Wire links
    R1C2.setPeer(R2, R2C1); R2C1.setPeer(R1, R1C2);
    R2C2.setPeer(R3, R3C1); R3C1.setPeer(R2, R2C2);
    R3C2.setPeer(R4, R4C1); R4C1.setPeer(R3, R3C2);

    // Endpoints per router
    TestEndpoint E1(R1, CRSF_ADDRESS_CRSF_TRANSMITTER);
    TestEndpoint E2(R2, CRSF_ADDRESS_CRSF_RECEIVER);
    TestEndpoint E3(R3, CRSF_ADDRESS_CRSF_TRANSMITTER);
    TestEndpoint E4(R4, CRSF_ADDRESS_CRSF_RECEIVER);
    R1.addEndpoint(&E1); R2.addEndpoint(&E2); R3.addEndpoint(&E3); R4.addEndpoint(&E4);

    // Radio -> FC flow
    crsf_ext_header_t radio_to_fc{};
    create_test_message(&radio_to_fc, CRSF_FRAMETYPE_DEVICE_PING, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_ADDRESS_RADIO_TRANSMITTER);
    R1.processMessage(&R1C1, (crsf_header_t *)&radio_to_fc);
    TEST_ASSERT_TRUE(R4C2.forwardMessageCalled);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, R4C2.lastForwardedMessage.dest_addr);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_RADIO_TRANSMITTER, R4C2.lastForwardedMessage.orig_addr);

    // FC -> Radio flow
    crsf_ext_header_t fc_to_radio{};
    create_test_message(&fc_to_radio, CRSF_FRAMETYPE_DEVICE_PING, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    R4.processMessage(&R4C2, (crsf_header_t *)&fc_to_radio);
    TEST_ASSERT_TRUE(R1C1.forwardMessageCalled);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, R1C1.lastForwardedMessage.orig_addr);
    TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_RADIO_TRANSMITTER, R1C1.lastForwardedMessage.dest_addr);

    // Broadcast ping and expect 4 replies observed near TX side
    crsf_ext_header_t broadcastPing{};
    create_test_message(&broadcastPing, CRSF_FRAMETYPE_DEVICE_PING, CRSF_ADDRESS_BROADCAST, CRSF_ADDRESS_RADIO_TRANSMITTER);
    R1C1.reset(); R1C2.reset(); R2C1.reset(); R2C2.reset(); R3C1.reset(); R3C2.reset(); R4C1.reset(); R4C2.reset();
    R1.processMessage(&R1C1, (crsf_header_t *)&broadcastPing);

    size_t observedAtTxSide = R1C1.forwardedMessages.size();
    TEST_ASSERT_EQUAL_INT(4, observedAtTxSide);

    bool sawNatOnTxr = false;
    for (const auto &msg : R2C2.forwardedMessages) {
        if (msg.orig_addr >= CRSF_ADDRESS_NAT_START && msg.orig_addr < CRSF_ADDRESS_NAT_END) { sawNatOnTxr = true; break; }
    }
    if (!sawNatOnTxr) {
        for (const auto &msg : R2C1.forwardedMessages) {
            if (msg.orig_addr >= CRSF_ADDRESS_NAT_START && msg.orig_addr < CRSF_ADDRESS_NAT_END) { sawNatOnTxr = true; break; }
        }
    }
    TEST_ASSERT_TRUE(sawNatOnTxr);
}

// --- Test Runner ---
int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_natMessage_rewrites_origin_and_updates_crc);
    RUN_TEST(test_deNatMessage_rewrites_origin_back_to_original_and_updates_crc);
    RUN_TEST(test_NAT_Chained_TX_RXR_TXR_RX);
    return UNITY_END();
}
