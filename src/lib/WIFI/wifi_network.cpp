#include "wifi_network.h"

#include <set>

#include "logging.h"
#include "common.h"
#include "options.h"
#include "WebContent.h"
#include "wifi_common.h"
#include "wifi_update.h"

static constexpr uint32_t STALE_WIFI_SCAN = 20000;
static uint32_t lastScanTimeMS = 0;

#if defined(PLATFORM_ESP8266)
static bool scanComplete = false;
#endif

static boolean isIp(const String& str)
{
    for (size_t i = 0; i < str.length(); i++)
    {
        int c = str.charAt(i);
        if (c != '.' && (c < '0' || c > '9'))
        {
            return false;
        }
    }
    return true;
}

static String toStringIp(const IPAddress& ip)
{
    String res = "";
    for (int i = 0; i < 3; i++)
    {
        res += String((ip >> (8 * i)) & 0xFF) + ".";
    }
    res += String(((ip >> 8 * 3)) & 0xFF);
    return res;
}

static String buildAccessPointMessage()
{
    return String("Access Point starting, please connect to access point '") + wifi_ap_ssid + "' with password '" + wifi_ap_password + "'";
}

static String buildStationConnectMessage()
{
    return String("Connecting to network '") + station_ssid + "', connect to http://" +
        wifi_hostname + ".local from a browser on that network";
}

static bool captivePortal(AsyncWebServerRequest *request)
{
    if (!isIp(request->host()) && request->host() != (String(wifi_hostname) + ".local"))
    {
        DBGLN("Request redirected to captive portal");
        request->redirect(String("http://") + toStringIp(request->client()->localIP()));
        return true;
    }
    return false;
}

static void WebUpdateSendContent(AsyncWebServerRequest *request)
{
    for (size_t i = 0; i < WEB_ASSETS_COUNT; i++) {
        if (request->url().equals(WEB_ASSETS[i].path)) {
            AsyncWebServerResponse *response = request->beginResponse(200, WEB_ASSETS[i].content_type, WEB_ASSETS[i].data, WEB_ASSETS[i].size);
            response->addHeader("Content-Encoding", "gzip");
            request->send(response);
            return;
        }
    }
    request->send(404, "text/plain", "File not found");
}

static void WebUpdateHandleRoot(AsyncWebServerRequest *request)
{
    if (captivePortal(request))
    {
        return;
    }
    wifiUpdateSetForceUpdate(request->hasArg("force"));
    if (connectionState == hardwareUndefined)
    {
        request->redirect("/index.html#hardware");
    }
    else
    {
        request->redirect("/index.html");
    }
}

static void sendNetworkScanResponse(AsyncWebServerRequest *request, const char *status, int numNetworks = -1)
{
    auto *response = new AsyncJsonResponse();
    JsonObject root = response->getRoot().to<JsonObject>();
    root["status"] = status;
    JsonArray networks = root["networks"].to<JsonArray>();

    if (numNetworks >= 0)
    {
        std::set<String> seenNetworks;
        for (int i = 0; i < numNetworks; i++)
        {
            String ssid = WiFi.SSID(i);
            DBGLN("found %s", ssid.c_str());
            if (ssid.length() > 0 && seenNetworks.insert(ssid).second)
            {
                (void)networks.add(ssid);
            }
        }
    }

    finalizeJsonResponse(request, response);
}

static void WebUpdateSendNetworks(AsyncWebServerRequest *request)
{
    int numNetworks = WiFi.scanComplete();
    if (numNetworks >= 0 && millis() - lastScanTimeMS < STALE_WIFI_SCAN) {
        DBGLN("Found %d networks", numNetworks);
        sendNetworkScanResponse(request, "ready", numNetworks);
    } else {
        if (WiFi.scanComplete() != WIFI_SCAN_RUNNING)
        {
#if defined(PLATFORM_ESP8266)
            scanComplete = false;
            WiFi.scanNetworksAsync([](int){
                scanComplete = true;
            });
#else
            WiFi.scanNetworks(true);
#endif
            lastScanTimeMS = millis();
        }
        sendNetworkScanResponse(request, "scanning");
    }
}

static void sendResponse(AsyncWebServerRequest *request, const String &msg, WiFiMode_t mode, bool includeOptions = false)
{
    sendJsonResponse(request, "ok", msg, [includeOptions](JsonObject root) {
        if (includeOptions)
        {
            appendOptionsJson(root);
        }
    });
    scheduleModeChange(mode, millis());
}

static void WebUpdateAccessPoint(AsyncWebServerRequest *request)
{
    DBGLN("Starting Access Point");
    sendResponse(request, buildAccessPointMessage(), WIFI_AP);
}

static void WebUpdateConnect(AsyncWebServerRequest *request)
{
    DBGLN("Connecting to network");
    sendResponse(request, buildStationConnectMessage(), WIFI_STA);
}

static int parseWifiAutoOnIntervalMillis(const String &onInterval)
{
    return (onInterval.isEmpty() ? -1 : onInterval.toInt()) * 1000;
}

static void WebUpdateSetHome(AsyncWebServerRequest *request)
{
    String ssid = request->arg("network");
    String password = request->arg("password");
    String onInterval = request->arg("wifi-on-interval");

    DBGLN("Setting network %s", ssid.c_str());
    copyStationCredentials(ssid.c_str(), password.c_str());
    const bool saveHomeNetwork = request->hasArg("save");
    if (saveHomeNetwork) {
        strlcpy(firmwareOptions.home_wifi_ssid, ssid.c_str(), sizeof(firmwareOptions.home_wifi_ssid));
        strlcpy(firmwareOptions.home_wifi_password, password.c_str(), sizeof(firmwareOptions.home_wifi_password));
        firmwareOptions.wifi_auto_on_interval = parseWifiAutoOnIntervalMillis(onInterval);
        saveOptions();
    }
    sendResponse(request, buildStationConnectMessage(), WIFI_STA, saveHomeNetwork);
}

static void WebUpdateForget(AsyncWebServerRequest *request)
{
    DBGLN("Forget network");
    String onInterval = request->arg("wifi-on-interval");
    firmwareOptions.home_wifi_ssid[0] = 0;
    firmwareOptions.home_wifi_password[0] = 0;
    firmwareOptions.wifi_auto_on_interval = parseWifiAutoOnIntervalMillis(onInterval);
    saveOptions();
    copyStationCredentials("", "");
    sendResponse(request, buildAccessPointMessage(), WIFI_AP, true);
}

static void WebUpdateHandleNotFound(AsyncWebServerRequest *request)
{
    if (captivePortal(request))
    {
        return;
    }
    String message = F("File Not Found\n\n");
    message += F("URI: ");
    message += request->url();
    message += F("\nMethod: ");
    message += (request->method() == HTTP_GET) ? "GET" : "POST";

    message += F("\nArguments: ");
    message += request->args();
    message += F("\n");

    for (uint8_t i = 0; i < request->args(); i++)
    {
        message += String(F(" ")) + request->argName(i) + F(": ") + request->arg(i) + F("\n");
    }
    AsyncWebServerResponse *response = request->beginResponse(404, "text/plain", message);
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Pragma", "no-cache");
    response->addHeader("Expires", "-1");
    request->send(response);
}

void registerWifiNetworkHandlers(AsyncWebServer &server)
{
    for (auto asset : WEB_ASSETS)
    {
        server.on(asset.path, WebUpdateSendContent);
    }
    server.on("/networks.json", WebUpdateSendNetworks);
    server.on("/sethome", WebUpdateSetHome);
    server.on("/forget", WebUpdateForget);
    server.on("/connect", WebUpdateConnect);
    server.on("/access", WebUpdateAccessPoint);

    server.on("/connecttest.txt", [](AsyncWebServerRequest *request) {
        request->redirect("http://logout.net");
    });
    server.on("/wpad.dat", [](AsyncWebServerRequest *request) {
        request->send(404);
    });
    server.on("/success.txt", [](AsyncWebServerRequest *request) {
        request->send(200);
    });

    const char* rootUris[] = {
        "/",
        "/generate_204",
        "/gen_204",
        "/library/test/success.html",
        "/hotspot-detect.html",
        "/connectivity-check.html",
        "/check_network_status.txt",
        "/ncsi.txt",
        "/canonical.html",
        "/fwlink",
        "/redirect"
    };

    for (const char* uri : rootUris)
        server.on(uri, WebUpdateHandleRoot);

    server.onNotFound(WebUpdateHandleNotFound);
}
