#pragma once

#include <ESPAsyncWebServer.h>

void registerWifiHardwareHandlers(AsyncWebServer &server);
uint8_t wifiHardwareGetDefinedVoltageSourceCount();
