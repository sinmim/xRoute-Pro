#ifndef XROUTE_WIFI_MANAGER_H
#define XROUTE_WIFI_MANAGER_H

#include <WiFi.h>
#include <functional>

class XrouteWiFiManager
{
public:
  void begin(const String &ssid, const String &pass, const String &hostname);
  void onConnected(std::function<void()> cb);
  void onDisconnected(std::function<void(int reason)> cb);

private:
  String _ssid, _pass;
  TaskHandle_t _reconnectTaskHandle = NULL;

  std::function<void()> _onConnected = nullptr;
  std::function<void(int)> _onDisconnected = nullptr;

  static void reconnectTask(void *param);
  void handleWiFiEvent(arduino_event_id_t event, arduino_event_info_t info);
  static bool isTargetSSIDAvailable(const String &targetSSID);
};

#endif
