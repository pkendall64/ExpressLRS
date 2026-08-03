#ifndef TX_USB_CONNECTOR_H
#define TX_USB_CONNECTOR_H
#include "CRSFConnector.h"

class TXUSBConnector final : public CRSFConnector
{
public:
    explicit TXUSBConnector(Stream *stream) : stream(stream) {}
    void forwardMessage(const crsf_header_t *message) override;
    uint8_t GetMaxPacketBytes() const override;

private:
    Stream *stream;
};

#endif //TX_USB_CONNECTOR_H
