#ifndef XROUTE_WIFI_SERVER_MANAGER_H
#define XROUTE_WIFI_SERVER_MANAGER_H

#include <WiFi.h>
#include <functional>

class XrouteWiFiServerManager
{
public:
  void begin(wifi_mode_t mode,
             const String &hostname,
             const String &sta_ssid = "", const String &sta_pass = "",
             const String &ap_ssid = "", const String &ap_pass = "");

  // --- Event Callbacks ---
  void onStaConnected(std::function<void()> cb);
  void onStaDisconnected(std::function<void(int reason)> cb);
  void onApReady(std::function<void()> cb);

  bool isStaConnected() const;

private:
  // Configuration
  String _hostname;
  String _sta_ssid;
  String _sta_pass;
  String _ap_ssid;
  String _ap_pass;
  wifi_mode_t _mode;

  // State
  bool _staConnectedFlg = false;
  TaskHandle_t _reconnectTaskHandle = NULL;

  // Callbacks
  std::function<void()> _onStaConnectedCb = nullptr;
  std::function<void(int)> _onStaDisconnectedCb = nullptr;
  std::function<void()> _onApReadyCb = nullptr;

  // Internal Methods
  void initializeMDNS();
  void handleWiFiEvent(arduino_event_id_t event, arduino_event_info_t info);
  static void reconnectTask(void *param);
  static bool isTargetSSIDAvailable(const String &targetSSID);
};

#endif