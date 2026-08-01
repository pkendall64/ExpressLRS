#pragma once

#include "device.h"
#include <cstddef>

class MAVLinkUplink
{
public:
    uint16_t free() const;
    uint16_t size() const;
    void push(const uint8_t *data, size_t size);
};

extern device_t MAVLinkUplink_device;
