#pragma once

#include "SerialIO.h"

extern SerialIO *serialIO;
extern SerialIO *serial1IO;

extern device_t Serial0_device;
#if defined(PLATFORM_ESP32)
extern device_t Serial1_device;
#endif

void sendImmediateRC();
void handleSerialIO();
void crsfRCFrameAvailable();
void crsfRCFrameMissed();
void serialPreConfigure();
void reconfigureSerial0();
