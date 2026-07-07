#include "device.h"

#if defined(PLATFORM_ESP32)
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Updater.h>
#define wifi_mode_t WiFiMode_t
#endif
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>

#include "POWERMGNT.h"
#include "common.h"
#include "devButton.h"
#include "hwTimer.h"
#include "logging.h"
#include "options.h"
#include "wifi_common.h"
#include "wifi_config.h"
#include "wifi_hardware.h"
#include "wifi_network.h"
#include "wifi_update.h"
#if defined(RADIO_LR1121)
#include "wifi_lr1121.h"
#endif

#if defined(TARGET_TX)
#include "wifi_joystick.h"
#else
#include "wifi_msp.h"
TcpMspConnector wifi2tcp;
#endif

#if defined(TARGET_RX) && defined(PLATFORM_ESP32)
#include "devVTXSPI.h"
#endif

const char VERSION[] = {LATEST_VERSION, 0};

char station_ssid[33];
static char station_password[65];

static bool wifiStarted = false;
bool webserverPreventAutoStart = false;

static wl_status_t laststatus = WL_IDLE_STATUS;
static volatile WiFiMode_t changeMode = WIFI_OFF;
static volatile unsigned long changeTime = 0;
volatile WiFiMode_t wifiMode = WIFI_OFF;

static constexpr byte DNS_PORT = 53;
static IPAddress netMsk(255, 255, 255, 0);
static DNSServer dnsServer;
static IPAddress ipAddress;

static AsyncWebServer server(80);
static bool servicesStarted = false;

void copyStationCredentials(const char *ssid, const char *password)
{
    strlcpy(station_ssid, ssid, sizeof(station_ssid));
    strlcpy(station_password, password, sizeof(station_password));
}

void scheduleModeChange(WiFiMode_t mode, unsigned long now)
{
    changeTime = now;
    changeMode = mode;
}

void setWifiUpdateMode()
{
    InBindingMode = false;
    setConnectionState(wifiUpdate);
}

static void stopWiFi()
{
    wifiStarted = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
#if defined(PLATFORM_ESP8266)
    WiFi.forceSleepBegin();
#endif
}

static bool initialize()
{
    stopWiFi();
    registerButtonFunction(ACTION_START_WIFI, []() {
        setWifiUpdateMode();
    });
    return true;
}

static void startWiFi(unsigned long now)
{
    if (wifiStarted)
    {
        return;
    }

    if (connectionState < FAILURE_STATES)
    {
        hwTimer::stop();
#if defined(TARGET_RX) && defined(PLATFORM_ESP32)
        disableVTxSpi();
#endif
        POWERMGNT::setPower(MinPower);
        setWifiUpdateMode();
        DBGLN("Stopping Radio");
        Radio.End();
    }

    DBGLN("Begin Webupdater");

    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    copyStationCredentials(firmwareOptions.home_wifi_ssid, firmwareOptions.home_wifi_password);
    scheduleModeChange(station_ssid[0] == 0 ? WIFI_AP : WIFI_STA, now);
    laststatus = WL_DISCONNECTED;
    wifiStarted = true;
}

static void startMDNS()
{
    if (!MDNS.begin(wifi_hostname))
    {
        DBGLN("Error starting mDNS");
        return;
    }

    String options = "-DAUTO_WIFI_ON_INTERVAL=" + (firmwareOptions.wifi_auto_on_interval == -1 ? "-1" : String(firmwareOptions.wifi_auto_on_interval / 1000));

#if defined(TARGET_TX)
    if (firmwareOptions.unlock_higher_power)
    {
        options += " -DUNLOCK_HIGHER_POWER";
    }
    options += " -DTLM_REPORT_INTERVAL_MS=" + String(firmwareOptions.tlm_report_interval);
    options += " -DFAN_MIN_RUNTIME=" + String(firmwareOptions.fan_min_runtime);
#endif

#if defined(TARGET_RX)
    if (firmwareOptions.lock_on_first_connection)
    {
        options += " -DLOCK_ON_FIRST_CONNECTION";
    }
    options += " -DRCVR_UART_BAUD=" + String(firmwareOptions.uart_baud);
#endif

    String instance = String(wifi_hostname) + "_" + WiFi.macAddress();
    instance.replace(":", "");
#if defined(PLATFORM_ESP8266)
    MDNS.setInstanceName(wifi_hostname);
    MDNSResponder::hMDNSService service = MDNS.addService(instance.c_str(), "http", "tcp", 80);
    MDNS.addServiceTxt(service, "vendor", "elrs");
    MDNS.addServiceTxt(service, "target", (const char *)&target_name[4]);
    MDNS.addServiceTxt(service, "device", (const char *)device_name);
    MDNS.addServiceTxt(service, "product", (const char *)product_name);
    MDNS.addServiceTxt(service, "version", VERSION);
    MDNS.addServiceTxt(service, "options", options.c_str());
    MDNS.addServiceTxt(service, "type", "rx");
    MDNS.setHostProbeResultCallback([instance](const char *p_pcDomainName, bool p_bProbeResult) {
        if (!p_bProbeResult)
        {
            WiFi.hostname(instance);
            MDNS.setInstanceName(instance);
        }
    });
#else
    MDNS.setInstanceName(instance);
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "vendor", "elrs");
    MDNS.addServiceTxt("http", "tcp", "target", (const char *)&target_name[4]);
    MDNS.addServiceTxt("http", "tcp", "device", (const char *)device_name);
    MDNS.addServiceTxt("http", "tcp", "product", (const char *)product_name);
    MDNS.addServiceTxt("http", "tcp", "version", VERSION);
    MDNS.addServiceTxt("http", "tcp", "options", options.c_str());
#if defined(TARGET_TX)
    MDNS.addServiceTxt("http", "tcp", "type", "tx");
#else
    MDNS.addServiceTxt("http", "tcp", "type", "rx");
#endif
#endif

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)
    MDNS.addService("elrs", "udp", JOYSTICK_PORT);
    MDNS.addServiceTxt("elrs", "udp", "device", (const char *)device_name);
    MDNS.addServiceTxt("elrs", "udp", "version", String(JOYSTICK_VERSION).c_str());
#endif
}

static void startServices()
{
    if (servicesStarted)
    {
#if defined(PLATFORM_ESP32)
        MDNS.end();
        startMDNS();
#endif
        return;
    }

    registerWifiNetworkHandlers(server);
    registerWifiConfigHandlers(server);
    registerWifiUpdateHandlers(server);
    registerWifiHardwareHandlers(server);
#if defined(TARGET_TX) && defined(PLATFORM_ESP32)
    WifiJoystick::RegisterHttpHandlers(server);
#endif
#if defined(RADIO_LR1121)
    addLR1121Handlers(server);
#endif

    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Max-Age", "600");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "POST,GET,OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

    server.begin();

    dnsServer.start(DNS_PORT, "*", ipAddress);
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);

    startMDNS();

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)
    WifiJoystick::StartJoystickService();
#endif
#if defined(TARGET_RX)
    wifi2tcp.begin();
#endif

    servicesStarted = true;
    DBGLN("HTTPUpdateServer ready! Open http://%s.local in your browser", wifi_hostname);
}

static void startAccessPointMode(unsigned long now)
{
    DBGLN("Changing to AP mode");
    WiFi.disconnect();
    wifiMode = WIFI_AP;
#if defined(PLATFORM_ESP32)
    WiFi.setHostname(wifi_hostname);
#endif
    WiFi.mode(wifiMode);
#if defined(PLATFORM_ESP8266)
    WiFi.setHostname(wifi_hostname);
#endif
    changeTime = now;
#if defined(PLATFORM_ESP8266)
    WiFi.setOutputPower(13.5);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#elif defined(PLATFORM_ESP32)
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
#endif
    WiFi.softAPConfig(ipAddress, ipAddress, netMsk);
    WiFi.softAP(wifi_ap_ssid, wifi_ap_password);
    startServices();
}

static void startStationMode(unsigned long now)
{
    DBGLN("Connecting to network '%s'", station_ssid);
    wifiMode = WIFI_STA;
#if defined(PLATFORM_ESP32)
    WiFi.setHostname(wifi_hostname);
#endif
    WiFi.mode(wifiMode);
#if defined(PLATFORM_ESP8266)
    WiFi.setHostname(wifi_hostname);
#endif
    changeTime = now;
#if defined(PLATFORM_ESP8266)
    WiFi.setOutputPower(13.5);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#elif defined(PLATFORM_ESP32)
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
#endif
    WiFi.begin(station_ssid, station_password);
    startServices();
}

static void HandleWebUpdate()
{
    unsigned long now = millis();
    wl_status_t status = WiFi.status();

    if (status != laststatus && wifiMode == WIFI_STA)
    {
        DBGLN("WiFi status %d", status);
        switch (status)
        {
        case WL_NO_SSID_AVAIL:
        case WL_CONNECT_FAILED:
        case WL_CONNECTION_LOST:
            scheduleModeChange(WIFI_AP, now);
            break;
        case WL_DISCONNECTED:
            changeTime = now;
            break;
        default:
            break;
        }
        laststatus = status;
    }
    if (status != WL_CONNECTED && wifiMode == WIFI_STA && (now - changeTime) > 30000)
    {
        scheduleModeChange(WIFI_AP, now);
        DBGLN("Connection failed %d", status);
    }
    if (changeMode != wifiMode && changeMode != WIFI_OFF && (now - changeTime) > 500)
    {
        switch (changeMode)
        {
        case WIFI_AP:
            startAccessPointMode(now);
            break;
        case WIFI_STA:
            startStationMode(now);
        default:
            break;
        }
#if defined(PLATFORM_ESP8266)
        MDNS.notifyAPChange();
#endif
        changeMode = WIFI_OFF;
    }

    if (servicesStarted)
    {
        dnsServer.processNextRequest();
#if defined(PLATFORM_ESP8266)
        MDNS.update();
#endif

#if defined(TARGET_TX) && defined(PLATFORM_ESP32)
        WifiJoystick::Loop(now);
#endif
    }
}

static int start()
{
    ipAddress.fromString(wifi_ap_address);
    return firmwareOptions.wifi_auto_on_interval;
}

static int event()
{
    if (connectionState == wifiUpdate || connectionState > FAILURE_STATES)
    {
        if (!wifiStarted)
        {
            startWiFi(millis());
            return DURATION_IMMEDIATELY;
        }
    }
    else if (wifiStarted)
    {
        stopWiFi();
    }
    return DURATION_IGNORE;
}

static int timeout()
{
    if (wifiStarted)
    {
        HandleWebUpdate();
#if defined(PLATFORM_ESP8266)
        if (!Update.isRunning())
            delay(1);
        return DURATION_IMMEDIATELY;
#else
        return 2;
#endif
    }

#if defined(TARGET_TX)
    if (firmwareOptions.wifi_auto_on_interval != -1 && webserverPreventAutoStart == false && connectionState < wifiUpdate && !wifiStarted)
    {
        DBGLN("No CRSF ever detected, starting WiFi");
        setWifiUpdateMode();
        return DURATION_IMMEDIATELY;
    }
#elif defined(TARGET_RX)
    if (firmwareOptions.wifi_auto_on_interval != -1 && !webserverPreventAutoStart && (connectionState == disconnected))
    {
        static bool pastAutoInterval = false;
        if (!InBindingMode || firmwareOptions.wifi_auto_on_interval >= 60000 || pastAutoInterval)
        {
            setWifiUpdateMode();
            return DURATION_IMMEDIATELY;
        }
        pastAutoInterval = true;
        return (60000 - firmwareOptions.wifi_auto_on_interval);
    }
#endif
    return DURATION_NEVER;
}

device_t WIFI_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONNECTION_CHANGED};
