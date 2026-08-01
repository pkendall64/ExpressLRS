#pragma once
#include "device.h"
#include "OTA.h"

void PackAirportData(OTA_Packet_s * otaPktPtr);
void UnpackAirportData(OTA_Packet_s const * otaPktPtr);

extern device_t Airport_device;