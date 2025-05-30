#ifndef XROUTE_ASYNC_WS_SERVER_H
#define XROUTE_ASYNC_WS_SERVER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <functional>
#include <atomic>

class XrouteAsyncWebSocketServer
{
public:
  /// How many WS clients are currently connected?
  size_t clientCount() const { return _clientCount.load(); }
  /// Convenience: “anyone there?”
  bool hasClients() const { return _ws->count() > 0; }

  void setSTA(const char *ssid, const char *pass);
  void initSTA();
  void setAP(const char *ssid, const char *pass);
  void setHostname(const char *name);
  void setPort(uint16_t wsPort = 81);
  void begin();
  void registerEvents();
  bool init(wifi_mode_t mode);

  void onCommand(std::function<void(const char *)> cb);
  void onJson(std::function<void(StaticJsonDocument<512> &)> cb);
  void sendToAll(const char *message);

  void printWiFiDetails();

  static void apMonitorTask(void *pv);
  void setStatusBuilder(std::function<void(StaticJsonDocument<512> &)> cb);
  void forceSwitchToAPMode();

private:
  std::atomic<size_t> _clientCount{0};
  const char *_staSsid = nullptr;
  const char *_staPass = nullptr;
  const char *_apSsid = nullptr;
  const char *_apPass = nullptr;
  const char *_host = "xroute";
  uint16_t _port = 81;
  static int currentMode;

  AsyncWebServer *_server = nullptr;
  AsyncWebSocket *_ws = nullptr;

  std::function<void(const char *)> _cmdCb;
  std::function<void(StaticJsonDocument<512> &)> _jsonCb;
  std::function<void(StaticJsonDocument<512> &)> _statusCb;

  StaticJsonDocument<512> _doc;
  char _outBuf[512];
};

#endif
