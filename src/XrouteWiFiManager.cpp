#include "XrouteWiFiManager.h"

void XrouteWiFiManager::begin(const String& ssid, const String& pass, const String& hostname) {
  _ssid = ssid;
  _pass = pass;

  WiFi.setHostname(hostname.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.begin(_ssid.c_str(), _pass.c_str());

  WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t info) {
    this->handleWiFiEvent(event, info);
  });
}

void XrouteWiFiManager::onConnected(std::function<void()> cb) {
  _onConnected = cb;
}

void XrouteWiFiManager::onDisconnected(std::function<void(int reason)> cb) {
  _onDisconnected = cb;
}

void XrouteWiFiManager::handleWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("[WiFi]: Got IP: " + WiFi.localIP().toString());
      if (_onConnected) _onConnected();
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi]: Disconnected from AP");
      Serial.println("[WiFi]: Disconnect Reason: " + String(info.wifi_sta_disconnected.reason));
      if (_onDisconnected) _onDisconnected(info.wifi_sta_disconnected.reason);

      if (_reconnectTaskHandle == NULL) {
        xTaskCreate(&XrouteWiFiManager::reconnectTask, "ReconnectTask", 4096, this, 1, &_reconnectTaskHandle);
      }
      break;

    default:
      break;
  }
}

bool XrouteWiFiManager::isTargetSSIDAvailable(const String& targetSSID) {
  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == targetSSID) return true;
  }
  return false;
}

void XrouteWiFiManager::reconnectTask(void* param) {
  XrouteWiFiManager* self = static_cast<XrouteWiFiManager*>(param);

  int retryCount = 0;
  const int maxRetry = 0; // 0 = infinite
  const uint32_t CONNECT_TIMEOUT = 10000;

  Serial.println("[WIFI RECONNECT TASK] : Waiting for SSID...");
  while (!isTargetSSIDAvailable(self->_ssid)) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  Serial.println("\n[WIFI RECONNECT TASK] : SSID found. Starting connection attempts");

  while (true) {
    Serial.println("[WIFI RECONNECT TASK] : Connecting to SSID: " + self->_ssid);
    WiFi.begin(self->_ssid.c_str(), self->_pass.c_str());

    uint32_t attemptStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - attemptStart) < CONNECT_TIMEOUT) {
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[WIFI RECONNECT TASK] : Connected");
      break;
    }

    Serial.println("[WIFI RECONNECT TASK] : Failed to connect. Retrying...");
    retryCount++;
    if (maxRetry > 0 && retryCount >= maxRetry) {
      Serial.println("[WIFI RECONNECT TASK] : Max retries reached. Giving up.");
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(3000)); // backoff
  }

  self->_reconnectTaskHandle = NULL;
  vTaskDelete(NULL);
}
