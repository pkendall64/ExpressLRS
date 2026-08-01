#pragma once

#include "device.h"

void forwardMAVLinkPayloadToUSB(const uint8_t *message, uint8_t size);

extern device_t TxUSBSerial_device;
