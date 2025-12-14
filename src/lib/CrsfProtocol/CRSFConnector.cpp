#include "targets.h"

#include "CRSF.h"
#include "CRSFConnector.h"
#include "logging.h"

uint8_t CRSFConnector::natBuffer[CRSF_MAX_PACKET_LEN] {};

void CRSFConnector::addDevice(const crsf_addr_e device_id)
{
    if (device_id != CRSF_ADDRESS_BROADCAST && devices.find(device_id) == devices.end())
    {
        devices.insert(device_id);
    }
}

bool CRSFConnector::forwardsTo(const crsf_addr_e device_id)
{
    return devices.find(device_id) != devices.end();
}

void CRSFConnector::registerNat(const crsf_addr_e device_id)
{
    crsf_addr_e natAddr = CRSF_ADDRESS_BROADCAST;
    for (uint8_t addr = CRSF_ADDRESS_NAT_START; addr < CRSF_ADDRESS_NAT_END; addr++)
    {
        natAddr = static_cast<crsf_addr_e>(addr);
        if (!forwardsTo(natAddr))
        {
            addDevice(natAddr);
            device_nats.push_back(static_cast<uint16_t>(device_id) << 8 | natAddr);
            break;
        }
    }
}

const crsf_ext_header_t *CRSFConnector::natMessage(const crsf_ext_header_t *message)
{
    if (shouldNatDevice(message->orig_addr)) {
        registerNat(message->orig_addr);
        for (const auto pair : device_nats)
        {
            if (message->orig_addr == pair >> 8)
            {
                const auto outMessage = (crsf_ext_header_t *)natBuffer;
                memcpy(natBuffer, message, message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES);
                outMessage->orig_addr = static_cast<crsf_addr_e>(pair & 0xFF);
                const uint8_t crc = CRSF::CRC.calc(natBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, message->frame_size - CRSF_TELEMETRY_CRC_LENGTH);
                natBuffer[message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES - CRSF_TELEMETRY_CRC_LENGTH] = crc;
                return outMessage;
            }
        }
    }
    return message;
}

const crsf_ext_header_t *CRSFConnector::deNatMessage(const crsf_ext_header_t *message) const
{
    for (const auto pair : device_nats)
    {
        if (message->dest_addr == (pair & 0xFF))
        {
            const auto outMessage = (crsf_ext_header_t *)natBuffer;
            memcpy(natBuffer, message, message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES);
            outMessage->dest_addr = static_cast<crsf_addr_e>(pair >> 8);
            const uint8_t crc = CRSF::CRC.calc(natBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, message->frame_size - CRSF_TELEMETRY_CRC_LENGTH);
            natBuffer[message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES - CRSF_TELEMETRY_CRC_LENGTH] = crc;
            return outMessage;
        }
    }
    return message;
}


void CRSFConnector::debugCRSF(const char *str, const crsf_header_t *message)
{
    DBGLN(str);
    DBGLN("dev:  %x", message->sync_byte);
    DBGLN("size: %x", message->frame_size);
    DBGLN("type: %x", message->type);
    if (message->type >= CRSF_FRAMETYPE_DEVICE_PING)
    {
        const auto ext = (crsf_ext_header_t *)message;
        DBGLN("dest: %x", ext->dest_addr);
        DBGLN("orig: %x", ext->orig_addr);
    }
}
