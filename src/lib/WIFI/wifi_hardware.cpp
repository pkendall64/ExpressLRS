#include "wifi_hardware.h"

#include "deferred.h"

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

#include "common.h"
#include "FHSS.h"
#include "POWERMGNT.h"
#include "options.h"
#include "wifi_common.h"
#include "devAnalogVbat.h"

#if defined(TARGET_RX)
#include "VbatCalibration.h"
#endif

static void putHardwareFile(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    static File file;
    static size_t bytes;
    if (!file || strcmp(&request->url().c_str()[1], file.name()) != 0)
    {
        file = LittleFS.open(request->url(), "w");
        bytes = 0;
    }
    file.write(data, len);
    bytes += len;
    if (bytes == total)
    {
        file.close();
    }
}

static void getHardwareFile(AsyncWebServerRequest *request)
{
    if (request->method() == HTTP_POST) {
        sendJsonResponse(request, "ok", "Hardware saved", [](JsonObject root) {
            JsonDocument hardware;
            deserializeJson(hardware, getHardware());
            root["hardware"] = hardware;
        });
    } else {
        request->send(200, "application/json", getHardware());
    }
}

uint8_t wifiHardwareGetDefinedVoltageSourceCount()
{
#if defined(TARGET_RX)
    uint8_t count = 0;
    if (hardware_pin(HARDWARE_vbat) != UNDEF_PIN)
        ++count;
#if defined(PLATFORM_ESP32)
    if (hardware_pin(HARDWARE_vsrc1) != UNDEF_PIN)
        ++count;
    if (hardware_pin(HARDWARE_vsrc2) != UNDEF_PIN)
        ++count;
    if (hardware_pin(HARDWARE_vsrc3) != UNDEF_PIN)
        ++count;
#endif
    return count;
#else
    return 0;
#endif
}

#if defined(TARGET_RX)
static void populateVoltageSampleJson(JsonObject root, const voltage_source_sample_t &sample)
{
    root["rawMax"] = sample.rawMax;
    root["adcMedian"] = sample.adcMedian;
    root["saturated"] = sample.saturated;
    root["hasReading"] = sample.hasReading;
}

static void SampleVoltageSources(AsyncWebServerRequest *request, JsonVariant &json)
{
    JsonArray requests = json["requests"].as<JsonArray>();
    if (requests.isNull())
    {
        request->send(400, "text/plain", "Voltage sample batch requests are required");
        return;
    }

    auto *response = new AsyncJsonResponse();
    JsonObject root = response->getRoot().to<JsonObject>();
    JsonObject samplesRoot = root["samples"].to<JsonObject>();

    bool sampledAny = false;
    Vbat_setCalibrationActive(true);
    for (JsonVariant requestItem : requests)
    {
        uint8_t sourceIdx = 0;
        const char *sourceId = requestItem["source"] | "";
        if (!VbatCalibration_findSource(sourceId, &sourceIdx) || !VbatCalibration_isSourceDefined(sourceIdx))
            continue;

        voltage_source_config_t source {};
        VbatCalibration_getSourceConfig(sourceIdx, &source);
        int atten = requestItem["atten"] | source.atten;
        uint8_t samples = requestItem["samples"] | 24;

        voltage_source_sample_t sample {};
        if (!VbatCalibration_sampleSource(sourceIdx, atten, samples, &sample))
            continue;

        JsonObject sampleRoot = samplesRoot[source.id].to<JsonObject>();
        populateVoltageSampleJson(sampleRoot, sample);
        sampledAny = true;
    }
    Vbat_setCalibrationActive(false);

    if (!sampledAny)
    {
        delete response;
        request->send(400, "text/plain", "No valid voltage sample batch requests");
        return;
    }

    finalizeJsonResponse(request, response);
}
#endif

static void HandleContinuousWave(AsyncWebServerRequest *request)
{
    if (request->hasArg("radio")) {
        SX12XX_Radio_Number_t radio = request->arg("radio").toInt() == 1 ? SX12XX_Radio_1 : SX12XX_Radio_2;

#if defined(RADIO_LR1121)
        bool setSubGHz = request->arg("subGHz").toInt() == 1;
#endif

        AsyncWebServerResponse *response = request->beginResponse(204);
        response->addHeader("Connection", "close");
        request->send(response);

        Radio.TXdoneCallback = [](){};
        Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

        POWERMGNT::init();
        POWERMGNT::setPower(POWERMGNT::getMinPower());

#if defined(RADIO_LR1121)
        Radio.startCWTest(setSubGHz ? FHSSconfig->freq_center : FHSSconfigDualBand->freq_center, radio);
#else
        Radio.startCWTest(FHSSconfig->freq_center, radio);
#if defined(RADIO_SX127X)
        deferExecutionMillis(50, [radio](){ Radio.cwRepeat(radio); });
#endif
#endif
        return;
    }

    auto *response = new AsyncJsonResponse();
    JsonObject root = response->getRoot().to<JsonObject>();
    root["radios"] = (GPIO_PIN_NSS_2 == UNDEF_PIN) ? 1 : 2;
    root["center"] = FHSSconfig->freq_center;
#if defined(RADIO_LR1121)
    root["center2"] = FHSSconfigDualBand->freq_center;
#endif
    finalizeJsonResponse(request, response);
}

void registerWifiHardwareHandlers(AsyncWebServer &server)
{
    server.on("/hardware.json", HTTP_GET | HTTP_POST, getHardwareFile, nullptr, putHardwareFile);
    server.on("/cw", HandleContinuousWave);
#if defined(TARGET_RX)
    server.addHandler(new AsyncCallbackJsonWebHandler("/voltage-sample", SampleVoltageSources));
#endif
}
