#include "SerialIO.h"
#include "crc.h"

class SerialSUMD : public SerialIO {
public:
    explicit SerialSUMD(Stream &stream) : SerialIO(&stream) { crc2Byte.init(16, 0x1021); }
    ~SerialSUMD() override {}

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t* data) override {}
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    Crc2Byte crc2Byte;
    void processBytes(uint8_t *bytes, uint16_t size) override {};
};
