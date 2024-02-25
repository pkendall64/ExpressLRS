#pragma once

#include "device.h"
#include "SerialIO.h"

extern SerialIO *serialIO;

void crsfRCFrameAvailable();
void crsfRCFrameMissed();
void serialPreConfigure();
void reconfigureSerial();

extern device_t Serial_device;
