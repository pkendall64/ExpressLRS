#include "wifi_update.h"

#if defined(PLATFORM_ESP32)
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <Update.h>
#else
#include <Updater.h>
#endif
#include <StreamString.h>

#include "logging.h"
#include "options.h"
#include "rxtx_intf.h"
#include "wifi_common.h"
#if defined(TARGET_TX) && defined(PLATFORM_ESP32)
#include "wifi_joystick.h"
#endif


static bool target_seen = false;
static uint8_t target_pos = 0;
static String target_found;
static bool target_complete = false;

static bool force_update = false;
static uint32_t totalSize = 0;
static size_t firmwareOffset = 0;

void wifiUpdateSetForceUpdate(bool enabled)
{
    force_update = enabled;
}

static void WebUploadResponseHandler(AsyncWebServerRequest *request)
{
    if (target_seen || Update.hasError()) {
        String msg;
        if (!Update.hasError() && Update.end()) {
            DBGLN("Update complete, rebooting");
            msg = "Update complete. ";
#if defined(TARGET_RX)
            msg += "Please wait for the LED to resume blinking before disconnecting power.";
#else
            msg += "Please wait for a few seconds while the device reboots.";
#endif
            scheduleRebootTime(200);
            sendJsonStatusResponse(request, "ok", msg);
            return;
        }

        StreamString p;
        if (Update.hasError()) {
            Update.printError(p);
        } else {
            p.println("Not enough data uploaded!");
        }
        p.trim();
        DBGLN("Failed to upload firmware: %s", p.c_str());
        sendJsonStatusResponse(request, "error", p);
        return;
    }

    String message = String("<b>Current target:</b> ") + (const char *)&target_name[4] + ".<br>";
    if (target_found.length() != 0) {
        message += "<b>Uploaded image:</b> " + target_found + ".<br/>";
    }
    message += "<br/>It looks like you are flashing firmware with a different name to the current firmware.  This sometimes happens because the hardware was flashed from the factory with an early version that has a different name. Or it may have even changed between major releases.";
    message += "<br/><br/>Please double check you are uploading the correct target, then proceed with 'Flash Anyway'.";
    sendJsonStatusResponse(request, "mismatch", message);
}

static void WebUploadDataHandler(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
{
    force_update = force_update || request->hasArg("force");
    if (index == 0) {
#if defined(TARGET_TX) && defined(PLATFORM_ESP32)
        WifiJoystick::StopJoystickService();
#endif
        size_t filesize = request->header("X-FileSize").toInt();
        DBGLN("Update: '%s' size %u", filename.c_str(), filesize);
#if defined(PLATFORM_ESP8266)
        Update.runAsync(true);
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        DBGLN("Free space = %u", maxSketchSpace);
        (void)maxSketchSpace;
#endif
        if (!Update.begin(filesize, U_FLASH)) {
            Update.printError(LOGGING_UART);
        }
        target_seen = false;
        target_found.clear();
        target_complete = false;
        target_pos = 0;
        totalSize = 0;
    }
    if (len) {
        DBGVLN("writing %d", len);
        if (Update.write(data, len) == len) {
            if (force_update || (totalSize == 0 && *data == 0x1F))
                target_seen = true;
            if (!target_seen) {
                for (size_t i = 0; i < len; i++) {
                    if (!target_complete && (target_pos >= 4 || target_found.length() > 0)) {
                        if (target_pos == 4) {
                            target_found.clear();
                        }
                        if (data[i] == 0 || target_found.length() > 50) {
                            target_complete = true;
                        } else {
                            target_found += (char)data[i];
                        }
                    }
                    if (data[i] == target_name[target_pos]) {
                        ++target_pos;
                        if (target_pos >= target_name_size) {
                            target_seen = true;
                        }
                    } else {
                        target_pos = 0;
                    }
                }
            }
            totalSize += len;
        } else {
            DBGLN("write failed to write %d", len);
        }
    }
}

static void WebUploadForceUpdateHandler(AsyncWebServerRequest *request)
{
    target_seen = true;
    if (request->arg("action").equals("confirm")) {
        WebUploadResponseHandler(request);
    } else {
#if defined(PLATFORM_ESP32)
        Update.abort();
#endif
        sendJsonStatusResponse(request, "ok", "Update cancelled");
    }
}

static size_t getFirmwareChunk(uint8_t *data, size_t len, size_t pos)
{
    uint8_t *dst;
    uint8_t alignedBuffer[7];
    if ((uintptr_t)data % 4 != 0)
    {
        dst = (uint8_t *)((uint32_t)alignedBuffer / 4 * 4);
        len = 4;
    }
    else
    {
        dst = data;
        len = constrain((len / 4) * 4, 4, SPI_FLASH_SEC_SIZE);
    }

    ESP.flashRead(firmwareOffset + pos, (uint32_t *)dst, len);

    if ((void *)dst != (void *)data)
    {
        for (unsigned b = len; b > 0; --b)
            *data++ = *dst++;
    }
    return len;
}

static void WebUpdateGetFirmware(AsyncWebServerRequest *request)
{
#if defined(PLATFORM_ESP32)
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        firmwareOffset = running->address;
    }
#endif
    const size_t firmwareTrailerSize = 4096;
    AsyncWebServerResponse *response = request->beginResponse("application/octet-stream", (size_t)ESP.getSketchSize() + firmwareTrailerSize, &getFirmwareChunk);
    String filename = String("attachment; filename=\"") + (const char *)&target_name[4] + "_" + VERSION + ".bin\"";
    response->addHeader("Content-Disposition", filename);
    request->send(response);
}


void registerWifiUpdateHandlers(AsyncWebServer &server)
{
    server.on("/update", HTTP_POST, WebUploadResponseHandler, WebUploadDataHandler);
    server.on("/update", HTTP_OPTIONS, corsPreflightResponse);
    server.on("/forceupdate", WebUploadForceUpdateHandler);
    server.on("/forceupdate", HTTP_OPTIONS, corsPreflightResponse);
    server.on("/firmware.bin", WebUpdateGetFirmware);
}
