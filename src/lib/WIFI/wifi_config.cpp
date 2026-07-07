#include "wifi_config.h"

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#if defined(PLATFORM_ESP32)
#include <esp_wifi.h>
#endif
#include "FHSS.h"

#include "common.h"
#include "config.h"
#include "logging.h"
#include "options.h"
#include "rxtx_intf.h"
#include "devButton.h"
#include "wifi_common.h"
#include "wifi_hardware.h"

#if defined(TARGET_TX)
extern void setButtonColors(uint8_t b1, uint8_t b2);
#endif

static const char *GetConfigUidType(const JsonObject json)
{
#if defined(TARGET_RX)
    if (config.GetBindStorage() == BINDSTORAGE_VOLATILE)
        return "Volatile";
    if (config.GetBindStorage() == BINDSTORAGE_RETURNABLE && config.IsOnLoan())
        return "Loaned";
    if (config.GetIsBound())
        return "Bound";
    return "Not Bound";
#else
    if (firmwareOptions.hasUID)
    {
        if (json["options"]["customised"] | false)
            return "Overridden";
        else
            return "Flashed";
    }
    return "Not set (using MAC address)";
#endif
}

static int8_t wifi_GetClientRssi()
{
    if (wifiMode == WIFI_STA)
        return WiFi.RSSI();

#if defined(PLATFORM_ESP32)
    wifi_sta_list_t staList;
    if (esp_wifi_ap_get_sta_list(&staList) == ESP_OK)
    {
        if (staList.num == 1)
            return staList.sta[0].rssi;
    }
#endif
    return 0;
}

static void populateConfigurationResponse(JsonObject root, bool exportMode)
{
    if (!exportMode)
    {
        appendOptionsJson(root);
    }

    const auto cfg = root["config"].to<JsonObject>();
    const auto uid = cfg["uid"].to<JsonArray>();
    copyArray(UID, UID_LEN, uid);

#if defined(TARGET_TX)
    int button_count = 0;
    if (GPIO_PIN_BUTTON != UNDEF_PIN)
        button_count = 1;
    if (GPIO_PIN_BUTTON2 != UNDEF_PIN)
        button_count = 2;
    for (int button = 0; button < button_count; button++)
    {
        const tx_button_color_t *buttonColor = config.GetButtonActions(button);
        const auto btn = cfg["button-actions"][button].to<JsonObject>();
        if (hardware_int(button == 0 ? HARDWARE_button_led_index : HARDWARE_button2_led_index) != -1) {
            btn["color"] = buttonColor->val.color;
        }
        for (int pos = 0; pos < button_GetActionCnt(); pos++)
        {
            const auto action = btn["action"][pos].to<JsonObject>();
            action["is-long-press"] = buttonColor->val.actions[pos].pressType ? true : false;
            action["count"] = buttonColor->val.actions[pos].count;
            action["action"] = buttonColor->val.actions[pos].action;
        }
    }
    if (exportMode)
    {
        cfg["fan-mode"] = config.GetFanMode();
        cfg["power-fan-threshold"] = config.GetPowerFanThreshold();
        cfg["motion-mode"] = config.GetMotionMode();

        const auto vtxAdmin = cfg["vtx-admin"].to<JsonObject>();
        vtxAdmin["band"] = config.GetVtxBand();
        vtxAdmin["channel"] = config.GetVtxChannel();
        vtxAdmin["pitmode"] = config.GetVtxPitmode();
        vtxAdmin["power"] = config.GetVtxPower();

        const auto backpack = cfg["backpack"].to<JsonObject>();
        backpack["disabled"] = config.GetBackpackDisable();
        backpack["dvr-start-delay"] = config.GetDvrStartDelay();
        backpack["dvr-stop-delay"] = config.GetDvrStopDelay();
        backpack["dvr-aux-channel"] = config.GetDvrAux();
        backpack["telemetry-mode"] = config.GetBackpackTlmMode();

        for (int model = 0; model < CONFIG_TX_MODEL_CNT; model++)
        {
            const model_config_t &modelConfig = config.GetModelConfig(model);
            String strModel(model);
            const auto modelJson = cfg["model"][strModel].to<JsonObject>();
            modelJson["packet-rate"] = modelConfig.rate;
            modelJson["telemetry-ratio"] = modelConfig.tlm;
            modelJson["switch-mode"] = modelConfig.switchMode;
            modelJson["link-mode"] = modelConfig.linkMode;
            modelJson["model-match"] = modelConfig.modelMatch;
            modelJson["tx-antenna"] = modelConfig.txAntenna;
            modelJson["ptr-start-chan"] = modelConfig.ptrStartChannel;
            modelJson["ptr-enable-chan"] = modelConfig.ptrEnableChannel;
            const auto power = cfg["power"].to<JsonObject>();
            power["max-power"] = modelConfig.power;
            power["dynamic-power"] = modelConfig.dynamicPower;
            power["boost-channel"] = modelConfig.boostChannel;
        }
    }
#endif

    if (!exportMode)
    {
        const auto settings = root["settings"].to<JsonObject>();
#if defined(TARGET_RX)
        cfg["serial-protocol"] = config.GetSerialProtocol();
#if defined(PLATFORM_ESP32)
        if ((GPIO_PIN_SERIAL1_RX != UNDEF_PIN && GPIO_PIN_SERIAL1_TX != UNDEF_PIN) || GPIO_PIN_PWM_OUTPUTS_COUNT > 0)
        {
            cfg["serial1-protocol"] = config.GetSerial1Protocol();
        }
#endif
        cfg["sbus-failsafe"] = config.GetFailsafeMode();
        cfg["modelid"] = config.GetModelId();
        cfg["force-tlm"] = config.GetForceTlmOff();
        cfg["vbind"] = config.GetBindStorage();
        for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
        {
            const auto channel = cfg["pwm"][ch].to<JsonObject>();
            channel["config"] = config.GetPwmChannel(ch)->raw;
            channel["pin"] = GPIO_PIN_PWM_OUTPUTS[ch];
            uint8_t features = 0;
            auto pin = GPIO_PIN_PWM_OUTPUTS[ch];
            if (!OPT_PWM_OUT_ONLY)
            {
                if (pin == U0TXD_GPIO_NUM) features |= 1;
                else if (pin == U0RXD_GPIO_NUM) features |= 2;
                else if (pin == GPIO_PIN_SCL) features |= 4;
                else if (pin == GPIO_PIN_SDA) features |= 8;
                else if (GPIO_PIN_SCL == UNDEF_PIN || GPIO_PIN_SDA == UNDEF_PIN) features |= 12;
            }
#if defined(PLATFORM_ESP32)
            if (pin != 0) features |= 16;
            if (!OPT_PWM_OUT_ONLY)
            {
                if (pin == GPIO_PIN_SERIAL1_RX) features |= 32;
                else if (pin == GPIO_PIN_SERIAL1_TX) features |= 64;
                else if ((GPIO_PIN_SERIAL1_RX == UNDEF_PIN || GPIO_PIN_SERIAL1_TX == UNDEF_PIN) &&
                         (!(features & 1) && !(features & 2))) features |= 96;
            }
#endif
            channel["features"] = features;
        }
        if (GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN)
        {
            settings["has_serial_pins"] = true;
        }
#endif
        settings["product_name"] = product_name;
        settings["lua_name"] = device_name;
        settings["uidtype"] = GetConfigUidType(root);
        settings["ssid"] = station_ssid;
        settings["mode"] = wifiMode == WIFI_STA ? "STA" : "AP";
        settings["wifi_dbm"] = wifi_GetClientRssi();
        settings["custom_hardware"] = hardware_flag(HARDWARE_customised);
        settings["target"] = &target_name[4];
        settings["version"] = VERSION;
        settings["git-commit"] = commit;
#if defined(TARGET_TX)
        settings["module-type"] = "TX";
#endif
#if defined(TARGET_RX)
        settings["module-type"] = "RX";
        settings["voltage_source_count"] = wifiHardwareGetDefinedVoltageSourceCount();
#endif
#if defined(RADIO_SX128X)
        settings["radio-type"] = "SX128X";
        settings["has_low_band"] = false;
        settings["has_high_band"] = true;
        settings["reg_domain_high"] = FHSSconfig->domain;
#elif defined(RADIO_SX127X)
        settings["radio-type"] = "SX127X";
        settings["has_low_band"] = true;
        settings["has_high_band"] = false;
        settings["reg_domain_low"] = FHSSconfig->domain;
#elif defined(RADIO_LR1121)
        settings["radio-type"] = "LR1121";
        settings["has_low_band"] = POWER_OUTPUT_VALUES_COUNT != 0;
        settings["has_high_band"] = POWER_OUTPUT_VALUES_DUAL_COUNT != 0;
        settings["reg_domain_low"] = FHSSconfig->domain;
        settings["reg_domain_high"] = FHSSconfigDualBand->domain;
#endif
    }
}

static void getOptionsFile(AsyncWebServerRequest *request)
{
    request->send(200, "application/json", getOptions());
}

static void HandleReboot(AsyncWebServerRequest *request)
{
    sendTextResponse(request, "Kill -9, no more CPU time!");
    scheduleRebootTime(200);
}

static void HandleReset(AsyncWebServerRequest *request)
{
    if (request->hasArg("hardware")) {
        LittleFS.remove("/hardware.json");
    }
    if (request->hasArg("options")) {
        LittleFS.remove("/options.json");
#if defined(TARGET_RX)
        config.SetModelId(255);
        config.SetForceTlmOff(false);
        config.Commit();
#endif
    }
    if (request->hasArg("lr1121")) {
        LittleFS.remove("/lr1121.txt");
    }
    if (request->hasArg("model") || request->hasArg("config")) {
        config.SetDefaults(true);
    }
    sendJsonStatusResponse(request, "ok", "Reset complete, rebooting...");
    scheduleRebootTime(100);
}

static void UpdateSettings(AsyncWebServerRequest *request, JsonVariant &json)
{
    if (firmwareOptions.flash_discriminator != json["flash-discriminator"].as<uint32_t>()) {
        request->send(409, "text/plain", "Mismatched device identifier, refresh the page and try again.");
        return;
    }

    File file = LittleFS.open("/options.json", "w");
    serializeJson(json, file);
    file.close();
    String options;
    serializeJson(json, options);
    setOptions(options);
    sendJsonResponse(request, "ok", "Options updated", [](JsonObject root) {
        appendOptionsJson(root);
    });
}

static void GetConfiguration(AsyncWebServerRequest *request)
{
    const bool exportMode = request->hasArg("export");
    auto *response = new AsyncJsonResponse();
    JsonObject root = response->getRoot().to<JsonObject>();
    populateConfigurationResponse(root, exportMode);
    finalizeJsonResponse(request, response);
}

#if defined(TARGET_TX)
static void UpdateConfiguration(AsyncWebServerRequest *request, JsonVariant &json)
{
    if (json["button-actions"].is<JsonVariant>()) {
        const JsonArray &array = json["button-actions"].as<JsonArray>();
        for (size_t button = 0; button < array.size(); button++)
        {
            tx_button_color_t action;
            for (int pos = 0; pos < button_GetActionCnt(); pos++)
            {
                action.val.actions[pos].pressType = array[button]["action"][pos]["is-long-press"];
                action.val.actions[pos].count = array[button]["action"][pos]["count"];
                action.val.actions[pos].action = array[button]["action"][pos]["action"];
            }
            action.val.color = array[button]["color"];
            config.SetButtonActions(button, &action);
        }
    }
    config.Commit();
    sendJsonResponse(request, "ok", "Configuration updated", [](JsonObject root) {
        populateConfigurationResponse(root, false);
    });
}

static void ImportConfiguration(AsyncWebServerRequest *request, JsonVariant &json)
{
    if (json["config"].is<JsonVariant>())
    {
        json = json["config"];
    }

    if (json["fan-mode"].is<JsonVariant>()) config.SetFanMode(json["fan-mode"]);
    if (json["power-fan-threshold"].is<JsonVariant>()) config.SetPowerFanThreshold(json["power-fan-threshold"]);
    if (json["motion-mode"].is<JsonVariant>()) config.SetMotionMode(json["motion-mode"]);

    if (json["vtx-admin"].is<JsonObject>())
    {
        const auto vtxAdmin = json["vtx-admin"].as<JsonObject>();
        if (vtxAdmin["band"].is<JsonVariant>()) config.SetVtxBand(vtxAdmin["band"]);
        if (vtxAdmin["channel"].is<JsonVariant>()) config.SetVtxChannel(vtxAdmin["channel"]);
        if (vtxAdmin["pitmode"].is<JsonVariant>()) config.SetVtxPitmode(vtxAdmin["pitmode"]);
        if (vtxAdmin["power"].is<JsonVariant>()) config.SetVtxPower(vtxAdmin["power"]);
    }

    if (json["backpack"].is<JsonVariant>())
    {
        const auto backpack = json["backpack"].as<JsonObject>();
        if (backpack["disabled"].is<JsonVariant>()) config.SetBackpackDisable(backpack["disabled"]);
        if (backpack["dvr-start-delay"].is<JsonVariant>()) config.SetDvrStartDelay(backpack["dvr-start-delay"]);
        if (backpack["dvr-stop-delay"].is<JsonVariant>()) config.SetDvrStopDelay(backpack["dvr-stop-delay"]);
        if (backpack["dvr-aux-channel"].is<JsonVariant>()) config.SetDvrAux(backpack["dvr-aux-channel"]);
        if (backpack["telemetry-mode"].is<JsonVariant>()) config.SetBackpackTlmMode(backpack["telemetry-mode"]);
    }

    if (json["model"].is<JsonVariant>())
    {
        for(JsonPair kv : json["model"].as<JsonObject>())
        {
            const uint8_t model = atoi(kv.key().c_str());
            const auto modelJson = kv.value().as<JsonObject>();

            config.SetModelId(model);
            if (modelJson["packet-rate"].is<JsonVariant>()) config.SetRate(modelJson["packet-rate"]);
            if (modelJson["telemetry-ratio"].is<JsonVariant>()) config.SetTlm(modelJson["telemetry-ratio"]);
            if (modelJson["switch-mode"].is<JsonVariant>()) config.SetSwitchMode(modelJson["switch-mode"]);
            if (modelJson["link-mode"].is<JsonVariant>()) config.SetLinkMode(modelJson["link-mode"]);
            if (modelJson["model-match"].is<JsonVariant>()) config.SetModelMatch(modelJson["model-match"]);
            if (modelJson["tx-antenna"].is<JsonVariant>()) config.SetAntennaMode(modelJson["tx-antenna"]);
            if (modelJson["ptr-start-chan"].is<JsonVariant>()) config.SetPTRStartChannel(modelJson["ptr-start-chan"]);
            if (modelJson["ptr-enable-chan"].is<JsonVariant>()) config.SetPTREnableChannel(modelJson["ptr-enable-chan"]);
            if (modelJson["power"].is<JsonVariant>())
            {
                if (modelJson["power"]["max-power"].is<JsonVariant>()) config.SetPower(modelJson["power"]["max-power"]);
                if (modelJson["power"]["dynamic-power"].is<JsonVariant>()) config.SetDynamicPower(modelJson["power"]["dynamic-power"]);
                if (modelJson["power"]["boost-channel"].is<JsonVariant>()) config.SetBoostChannel(modelJson["power"]["boost-channel"]);
            }
            config.Commit();
        }
    }

    UpdateConfiguration(request, json);
}

static void WebUpdateButtonColors(AsyncWebServerRequest *request, JsonVariant &json)
{
    int button1Color = json[0].as<int>();
    int button2Color = json[1].as<int>();
    DBGLN("%d %d", button1Color, button2Color);
    setButtonColors(button1Color, button2Color);
    request->send(200);
}
#else
static void JsonUidToConfig(JsonVariant &json)
{
    const auto juid = json["uid"].as<JsonArray>();
    size_t juidLen = constrain(juid.size(), 0, UID_LEN);
    uint8_t newUid[UID_LEN] = { 0 };
    copyArray(juid, &newUid[UID_LEN-juidLen], juidLen);

    if (memcmp(newUid, config.GetUID(), UID_LEN) != 0)
    {
        config.SetUID(newUid);
        config.Commit();
        memcpy(UID, newUid, UID_LEN);
    }
}

static void UpdateConfiguration(AsyncWebServerRequest *request, JsonVariant &json)
{
    uint8_t protocol = json["serial-protocol"] | 0;
    config.SetSerialProtocol((eSerialProtocol)protocol);

#if defined(PLATFORM_ESP32)
    uint8_t protocol1 = json["serial1-protocol"] | 0;
    config.SetSerial1Protocol((eSerial1Protocol)protocol1);
#endif

    uint8_t failsafe = json["sbus-failsafe"] | 0;
    config.SetFailsafeMode((eFailsafeMode)failsafe);

    long modelid = json["modelid"] | 255;
    if (modelid < 0 || modelid > 63) modelid = 255;
    config.SetModelId((uint8_t)modelid);

    long forceTlm = json["force-tlm"] | false;
    config.SetForceTlmOff(forceTlm != 0);

    config.SetBindStorage((rx_config_bindstorage_t)(json["vbind"] | 0));
    JsonUidToConfig(json);

    JsonArray pwm = json["pwm"].as<JsonArray>();
    for(uint32_t channel = 0; channel < pwm.size(); channel++)
    {
        rx_config_pwm_t pwmChannel;
        pwmChannel.raw = pwm[channel];
        if (OPT_PWM_OUT_ONLY &&
            (pwmChannel.val.mode == somSerial || pwmChannel.val.mode == somSCL || pwmChannel.val.mode == somSDA ||
             pwmChannel.val.mode == somSerial1RX || pwmChannel.val.mode == somSerial1TX))
        {
            pwmChannel.val.mode = som50Hz;
        }
        config.SetPwmChannelRaw(channel, pwmChannel.raw);
    }

    config.Commit();
    sendJsonResponse(request, "ok", "Configuration updated", [](JsonObject root) {
        populateConfigurationResponse(root, false);
    });
}
#endif

void registerWifiConfigHandlers(AsyncWebServer &server)
{
    server.on("/config", HTTP_GET, GetConfiguration);
    server.on("/options.json", HTTP_GET, getOptionsFile);
    server.on("/reboot", HandleReboot);
    server.on("/reset", HandleReset);
    server.addHandler(new AsyncCallbackJsonWebHandler("/config", UpdateConfiguration));
    server.addHandler(new AsyncCallbackJsonWebHandler("/options.json", UpdateSettings));
#if defined(TARGET_TX)
    server.addHandler(new AsyncCallbackJsonWebHandler("/buttons", WebUpdateButtonColors));
    auto *handler = new AsyncCallbackJsonWebHandler("/import", ImportConfiguration);
    handler->setMaxContentLength(32768);
    server.addHandler(handler);
#endif
}
