#ifndef SRC_CRSF_H
#define SRC_CRSF_H
#include "crc.h"
#include "crsf_protocol.h"

class CRSF {
public:
    virtual ~CRSF() = default;

    /**
     * Routes a CRSF message to a connector that is known to deliver messages to the given device id.
     *
     * The message is delivered to the connector responsible for the provided destination address. For broadcast messages
     * or messages with an unknown destination, the message is forwarded to all other connectors.
     *
     * @param destination The device_id of the target for this message.
     * @param message Pointer to the CRSF message header structure containing the message data.
     */
    virtual void deliverMessageTo(crsf_addr_e destination, const crsf_header_t *message) const = 0;

    /**
     * Sets the extended header fields and calculates the CRC for a CRSF frame.
     *
     * This function populates the extended header of a CRSF frame with the specified frame type,
     * size, and destination address. It also sets the originating address using the device ID
     * and calculates the CRC for the frame.
     *
     * @param frame Pointer to the CRSF extended header structure that will be populated and updated.
     * @param frameType The type of the CRSF frame to be set in the extended header.
     * @param frameSize The size of the CRSF frame, including payload, header (type, destination, origin, and CRC fields).
     * @param destAddr The destination address for the CRSF frame, as defined in the CRSF addressing scheme.
     * @param origAddr
     */
    virtual void SetExtendedHeaderAndCrc(crsf_ext_header_t *frame, crsf_frame_type_e frameType, uint8_t frameSize, crsf_addr_e destAddr, crsf_addr_e origAddr) const = 0;

    /**
     * Find the largest single packet size supported by the connector that delivers to the specified device Id.
     * This is used when chunking responses to parameter requests.
     *
     * @param origin The device Id that we are wanting to send packets to.
     * @return The maximum packet size that we can send in a single go.
     */
    virtual uint8_t getConnectorMaxPacketSize(crsf_addr_e origin) const = 0;

    static const GENERIC_CRC8 CRC;
};

#endif // SRC_CRSF_H
