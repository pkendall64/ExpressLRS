#pragma once

#include "SerialUplink.h"
#include "targets.h"

class BackpackSerial
{
public:
    bool initialize();
    size_t available();
    void poll(SerialUplink &uplink);
};
