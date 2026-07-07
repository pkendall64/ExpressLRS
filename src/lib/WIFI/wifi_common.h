#pragma once

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "options.h"

#if defined(PLATFORM_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

extern const char VERSION[];
extern char station_ssid[33];
extern volatile WiFiMode_t wifiMode;
void scheduleModeChange(WiFiMode_t mode, unsigned long now);
void copyStationCredentials(const char *ssid, const char *password);

template <typename PopulateBody>
void sendJsonResponse(AsyncWebServerRequest *request, const char *status, const String &msg, PopulateBody populateBody)
{
    auto *response = new AsyncJsonResponse();
    JsonObject root = response->getRoot().to<JsonObject>();
    root["status"] = status;
    root["msg"] = msg;
    populateBody(root);
    response->setLength();
    response->addHeader("Connection", "close");
    request->send(response);
}

inline void sendTextResponse(AsyncWebServerRequest *request, const String &msg, int status = 200)
{
    AsyncWebServerResponse *response = request->beginResponse(status, "text/plain", msg);
    response->addHeader("Connection", "close");
    request->send(response);
}

inline void sendJsonStatusResponse(AsyncWebServerRequest *request, const char *status, const String &msg)
{
    sendJsonResponse(request, status, msg, [](JsonObject){});
}

inline void finalizeJsonResponse(AsyncWebServerRequest *request, AsyncJsonResponse *response)
{
    response->setLength();
    request->send(response);
}

inline void appendOptionsJson(JsonObject root)
{
    JsonDocument options;
    deserializeJson(options, getOptions());
    root["options"] = options;
}

inline void corsPreflightResponse(AsyncWebServerRequest *request)
{
    AsyncWebServerResponse *response = request->beginResponse(204, "text/plain");
    request->send(response);
}
