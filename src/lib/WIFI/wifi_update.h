#pragma once

#include <ESPAsyncWebServer.h>

void registerWifiUpdateHandlers(AsyncWebServer &server);
void wifiUpdateSetForceUpdate(bool enabled);
