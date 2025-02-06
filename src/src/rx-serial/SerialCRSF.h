#include "SerialIO.h"

class SerialCRSF : public SerialIO {
public:
    explicit SerialCRSF(HardwareSerial &stream) : SerialIO(&stream) {}
    ~SerialCRSF() override {}

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void queueMSPFrameTransmission(uint8_t* data) override;
    void queueLinkStatisticsPacket() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    bool sendImmediateRC() override { return true; }

private:
    void processBytes(uint8_t *bytes, uint16_t size) override;
};
