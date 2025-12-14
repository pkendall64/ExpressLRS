#ifndef CRSF_CONNECTOR_H
#define CRSF_CONNECTOR_H

#include "crsf_protocol.h"

#include <set>
#include <vector>

/**
 * @class CRSFConnector
 *
 * @brief Abstract interface for integrating and managing connected CRSF devices.
 *
 * This class provides a mechanism to register devices that are reachable from this connector.
 * Derived classes must implement the pure virtual method `forwardMessage` to handle
 * the specifics of message forwarding.
 */
class CRSFConnector {
public:
    CRSFConnector() = default;
    virtual ~CRSFConnector() = default;

    /**
     * @brief Adds a device that is reachable via the connector to it's known device list.
     *
     * The connector may also use this information to build a NAT table which will be
     * used to rewrite addresses for devices that require it.
     *
     * @param device_id The CRSF address of the device to add.
     */
    void addDevice(crsf_addr_e device_id);

    /**
     * @brief Checks whether the connector is responsible for forwarding messages
     * to a specified CRSF device.
     *
     * @param device_id The CRSF address of the device to check.
     * @return True if the connector forwards messages to the specified device, false otherwise.
     */
    bool forwardsTo(crsf_addr_e device_id);

    /**
     * @brief Forwards a CRSF message to its destination device.
     *
     * This pure virtual method should be implemented by derived classes
     * to handle the specifics of forwarding a CRSF message to the intended device.
     *
     * It is up to the implementation how it performs the actual forwarding.
     * For some connectors, they may translate the message to a different format or
     * may decide not to forward the message at all.
     *
     * @param message A pointer to the CRSF message header structure to be forwarded.
     */
    virtual void forwardMessage(const crsf_header_t *message) = 0;

    /**
     * @brief Retrieves the maximum allowable packet size for the CRSF connector.
     *
     * The size is used to set the chunking size for parameter read responses.
     *
     * @return The maximum packet size in bytes supported by the connector.
     */
    virtual uint8_t GetMaxPacketBytes() const { return CRSF_MAX_PACKET_LEN; }

    /**
     * @brief Logs debugging information for a CRSF message.
     *
     * This method outputs details about a given CRSF message, including
     * the device address, frame size, and type. If the message type
     * corresponds to an extended header frame (e.g., `CRSF_FRAMETYPE_DEVICE_PING` or higher),
     * additional fields such as destination and origin addresses are also logged.
     *
     * @param str A prefix or context string to include in the debug output.
     * @param message A pointer to the CRSF message header structure containing the data to be logged.
     */
    static void debugCRSF(const char * str, const crsf_header_t * message);

    /**
     * @brief Determines whether the specified CRSF device requires NAT (Network Address Translation).
     *
     * This method checks if the device identified by the given address should have its
     * address translated to or from a NAT alias. It allows connectors to conditionally
     * decide which devices are subject to NAT.
     *
     * @param device_id The CRSF address of the device to check.
     * @return True if NAT should be applied to the specified device, false otherwise.
     */
    virtual bool shouldNatDevice(crsf_addr_e device_id) { return false; }

    /**
     * @brief Processes a CRSF extended header message for network address translation (NAT).
     *
     * This method analyzes the provided CRSF extended header message to determine if the
     * source address requires NAT translation. If NAT is required, it modifies the message
     * appropriately and updates the address and checksum. If no translation is required,
     * the original message is returned.
     *
     * @param message A pointer to the CRSF extended header message to be processed.
     * @return A pointer to the modified message after applying NAT translation, or the
     *         original message if no translation was applied.
     */
    const crsf_ext_header_t *natMessage(const crsf_ext_header_t *message);

    /**
     * @brief Processes a CRSF message to perform deNAT (Network Address Translation) and
     * updates the destination address if applicable.
     *
     * This method checks the incoming CRSF message against known device NAT mappings and
     * modifies the destination address based on the mapping. If the mapping applies, a modified
     * copy of the message is returned. If no mapping is found, the original message is returned.
     *
     * @param message A pointer to the CRSF message (crsf_ext_header_t) to be processed.
     *                This message contains the original destination address and other relevant
     *                frame data.
     *
     * @return A pointer to a modified CRSF message if a NAT mapping applies, or the original
     *         message pointer if no mapping is found. The returned message resides in a local
     *         buffer, and its integrity must be maintained by the caller to avoid overwrites.
     */
    const crsf_ext_header_t *deNatMessage(const crsf_ext_header_t *message) const;

    // Test-only helper: allow unit tests to seed NAT mappings directly.
    // Each entry is a 16-bit pair: upper 8 bits = original, lower 8 bits = NAT alias.
    // This is intentionally exposed for tests to validate nat/deNat behavior.
    void test_addNatMapping(crsf_addr_e original, crsf_addr_e alias) { device_nats.push_back((uint16_t(original) << 8) | uint16_t(alias)); }

private:
    std::set<crsf_addr_e> devices;
    std::vector<uint16_t> device_nats; // upper=original, lower=NAT
    static uint8_t natBuffer[CRSF_MAX_PACKET_LEN];

    void registerNat(crsf_addr_e device_id);
};

#endif //CRSF_CONNECTOR_H
