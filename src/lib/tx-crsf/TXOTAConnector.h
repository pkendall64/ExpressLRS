#ifndef TX_OTA_CONNECTOR_H
#define TX_OTA_CONNECTOR_H

#include "CRSFConnector.h"
#include "device.h"

class TXOTAConnector final : public CRSFConnector {
public:
    TXOTAConnector();

    void forwardMessage(const crsf_header_t *message) override;
};

extern device_t CRSFUplink_device;

#endif //TX_OTA_CONNECTOR_H
