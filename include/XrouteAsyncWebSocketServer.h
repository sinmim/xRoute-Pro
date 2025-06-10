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
  bool updatingFlg;
  uint32_t updateLen;
  uint32_t updateProgress;
  uint32_t getUpdateLen()
  {
    return updateLen;
  }
  void startUpdate(uint32_t len)
  {
    updateProgress = 0;
    updateLen = len;
    updatingFlg = true;
  }
  void endUpdate()
  {
    updateProgress = 0;
    updateLen = 0;
    updatingFlg = false;
  }
  float getProgress()
  {
    return (float)updateProgress / (float)updateLen;
  }
  bool isUpdating()
  {
    return updatingFlg;
  }

  /// How many WS clients are currently connected?
  size_t clientCount() const { return _clientCount.load(); }
  /// Convenience: “anyone there?”
  bool hasClients() const { return _ws->count() > 0; }

  void setSTA(const char *ssid, const char *pass);
  void setAP(const char *ssid, const char *pass);
  void setHostname(const char *name);
  void setPort(uint16_t wsPort = 81);
  void begin(wifi_mode_t mode);
  wifi_mode_t switchMode(wifi_mode_t mode);
  wifi_mode_t getMode();

  void registerEvents();
  bool init(wifi_mode_t mode);
  void onUpdate(std::function<void(const char *, int len)> cb)
  {
    _updateCb = cb;
  }
  void onCommand(std::function<void(const char *)> cb);
  void onJson(std::function<void(StaticJsonDocument<4096> &)> cb);
  void sendToAll(const char *message);
  void sendToClient(const char *message, AsyncWebSocketClient *client);
  void sendToThisClient(const char *message);
  void SendToAllExcludeClient(const char *message, AsyncWebSocketClient *exCliant);

  void printWiFiDetails();

  static void apMonitorTask(void *pv);
  void setStatusBuilder(std::function<void(StaticJsonDocument<4096> &)> cb);
  AsyncWebSocketClient *getCliant() { return LastClient; }

private:
  static wifi_mode_t currentMode;
  std::atomic<size_t> _clientCount{0};
  const char *_staSsid = nullptr;
  const char *_staPass = nullptr;
  const char *_apSsid = nullptr;
  const char *_apPass = nullptr;
  const char *_host = "xroute";
  uint16_t _port = 81;
  AsyncWebSocketClient *LastClient;
  // client vector
  std::vector<AsyncWebSocketClient *> _clients;
  std::mutex _clientsLock; // or use portENTER_CRITICAL / portEXIT_CRITICAL since FreeRTOS

  AsyncWebServer *_server = nullptr;
  AsyncWebSocket *_ws = nullptr;

  std::function<void(const char *)> _cmdCb;
  std::function<void(StaticJsonDocument<4096> &)> _jsonCb;
  std::function<void(StaticJsonDocument<4096> &)> _statusCb;
  std::function<void(const char *, int len)> _updateCb;

  StaticJsonDocument<4096> _doc;
  // char _outBuf[2048];
};

#endif
