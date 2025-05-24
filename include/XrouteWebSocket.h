// XrouteWebSocket.h
#ifndef XROUTE_WEBSOCKET_H
#define XROUTE_WEBSOCKET_H

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <functional>

/*
an enum for all the wifi possible states for example 

*/
class XrouteWebSocket
{
public:
  // Configuration
  void setSTA(const char *ssid, const char *pass);
  void setAP(const char *ssid, const char *pass);
  void setHostname(const char *name);
  void setPort(uint16_t wsPort = 81);

  // Register callback for connection events
  void onConnect(std::function<void(uint8_t)> cb);
  void onDisconnect(std::function<void(uint8_t)> cb);
  // Register callback for incoming commands
  void onCommand(std::function<void(const char *)> cb);
  // Register callback to build outgoing status JSON
  void onBuildStatus(std::function<void(StaticJsonDocument<512> &)> cb);

  // Public send API
  void sendText(uint8_t clientId, const char *message);
  void sendToAll(const char *message);
  void broadcastStatus();

  // Fully encapsulated start: Wi-Fi, mDNS, WebSocket, RTOS task
  void start(UBaseType_t priority = 1, BaseType_t core = 1);

private:
  // Wi-Fi settings
  const char *_staSsid = nullptr;
  const char *_staPass = nullptr;
  const char *_apSsid = nullptr;
  const char *_apPass = nullptr;
  const char *_host = "xroute";
  uint16_t _port = 81;

  WebSocketsServer _ws{_port};
  std::function<void(uint8_t)> _connectCb;
  std::function<void(uint8_t)> _disconnectCb;
  std::function<void(const char *)> _cmdCb;
  std::function<void(StaticJsonDocument<512> &)> _statusCb;
  StaticJsonDocument<512> _doc;
  char _outBuf[512];

  // Task entry
  static void _taskEntry(void *selfPtr)
  {
    static_cast<XrouteWebSocket *>(selfPtr)->_run();
  }
  void _run();
  void _handleEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
};

#endif // XROUTE_WEBSOCKET_H
