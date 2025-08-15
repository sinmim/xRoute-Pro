#include "XrouteWiFiServerManager.h"
#include <ESPmDNS.h>

void XrouteWiFiServerManager::begin(wifi_mode_t mode,
                                    const String &hostname,
                                    const String &sta_ssid, const String &sta_pass,
                                    const String &ap_ssid, const String &ap_pass)
{
    _mode = mode;
    _hostname = hostname;
    _sta_ssid = sta_ssid;
    _sta_pass = sta_pass;
    _ap_ssid = ap_ssid;
    _ap_pass = ap_pass;

    WiFi.setHostname(_hostname.c_str());

    // IMPORTANT: We manage our own reconnection, so disable the default.
    WiFi.setAutoReconnect(false);

    // Register our event handler BEFORE starting any WiFi operations
    WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t info)
                 { this->handleWiFiEvent(event, info); });

    WiFi.mode(_mode);

    if (_mode == WIFI_AP || _mode == WIFI_AP_STA)
    {
        Serial.println("[WiFi Mgr] Starting AP mode...");
        IPAddress local_IP(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_IP, gateway, subnet);
        WiFi.softAP(_ap_ssid.c_str(), _ap_pass.c_str());
    }

    if (_mode == WIFI_STA || _mode == WIFI_AP_STA)
    {
        Serial.println("[WiFi Mgr] Starting STA mode connection...");
        WiFi.begin(_sta_ssid.c_str(), _sta_pass.c_str());
    }
}

void XrouteWiFiServerManager::initializeMDNS()
{
    if (MDNS.begin(_hostname.c_str()))
    {
        MDNS.addService("ws", "tcp", 81); // Assuming WS is on port 81
        Serial.println("[WiFi Mgr] mDNS responder started: " + _hostname + ".local");
    }
    else
    {
        Serial.println("[WiFi Mgr] Error starting mDNS.");
    }
}

void XrouteWiFiServerManager::handleWiFiEvent(arduino_event_id_t event, arduino_event_info_t info)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_AP_START:
        Serial.println("[WiFi Mgr] AP Started. IP: " + WiFi.softAPIP().toString());
        initializeMDNS();
        if (_onApReadyCb)
        {
            _onApReadyCb();
        }
        break;

    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("[WiFi Mgr] STA Started.");
        break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        _staConnectedFlg = true;
        Serial.println("[WiFi Mgr] STA Connected. IP: " + WiFi.localIP().toString());
        // If in APSTA mode, mDNS is already started by AP_START
        if (_mode == WIFI_STA)
        {
            initializeMDNS();
        }
        if (_onStaConnectedCb)
        {
            _onStaConnectedCb();
        }
        break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        _staConnectedFlg = false;
        Serial.println("[WiFi Mgr] STA Disconnected. Reason: " + String(info.wifi_sta_disconnected.reason));
        if (_onStaDisconnectedCb)
        {
            _onStaDisconnectedCb(info.wifi_sta_disconnected.reason);
        }
        // If the task isn't already running, start it.
        if (_reconnectTaskHandle == NULL)
        {
            Serial.println("[WiFi Mgr] Starting reconnect task...");
            xTaskCreate(&XrouteWiFiServerManager::reconnectTask, "ReconnectTask", 4096, this, 1, &_reconnectTaskHandle);
        }
        break;

    default:
        break;
    }
}

bool XrouteWiFiServerManager::isTargetSSIDAvailable(const String &targetSSID)
{
    int n = WiFi.scanNetworks(false, true);
    for (int i = 0; i < n; ++i)
    {
        if (WiFi.SSID(i) == targetSSID)
        {
            return true;
        }
    }
    return false;
}

void XrouteWiFiServerManager::reconnectTask(void *param)
{
    XrouteWiFiServerManager *self = static_cast<XrouteWiFiServerManager *>(param);

    Serial.println("[Reconnect Task] Started. Waiting for SSID...");

    while (true)
    {
        if (isTargetSSIDAvailable(self->_sta_ssid))
        {
            Serial.println("\n[Reconnect Task] Target SSID found. Attempting to connect...");
            WiFi.begin(self->_sta_ssid.c_str(), self->_sta_pass.c_str());

            // Wait for the event handler to signal a connection.
            // If we disconnect again, the event handler will re-launch this task.
            // This task's job is just to initiate the connection attempt.
            break;
        }
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    Serial.println("[Reconnect Task] Handing over to event system. Exiting task.");
    self->_reconnectTaskHandle = NULL;
    vTaskDelete(NULL);
}

// --- Public Method Implementations ---

void XrouteWiFiServerManager::onStaConnected(std::function<void()> cb) { _onStaConnectedCb = cb; }
void XrouteWiFiServerManager::onStaDisconnected(std::function<void(int)> cb) { _onStaDisconnectedCb = cb; }
void XrouteWiFiServerManager::onApReady(std::function<void()> cb) { _onApReadyCb = cb; }

bool XrouteWiFiServerManager::isStaConnected() const
{
    return _staConnectedFlg;
}