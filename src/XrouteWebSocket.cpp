// XrouteWebSocket.cpp
#include "XrouteWebSocket.h"

void XrouteWebSocket::setSTA(const char *ssid, const char *pass)
{
  _staSsid = ssid;
  _staPass = pass;
}
void XrouteWebSocket::setAP(const char *ssid, const char *pass)
{
  _apSsid = ssid;
  _apPass = pass;
}
void XrouteWebSocket::setHostname(const char *name) { _host = name; }
void XrouteWebSocket::setPort(uint16_t wsPort) { _port = wsPort; }

void XrouteWebSocket::onConnect(std::function<void(uint8_t)> cb) { _connectCb = cb; }
void XrouteWebSocket::onDisconnect(std::function<void(uint8_t)> cb) { _disconnectCb = cb; }
void XrouteWebSocket::onCommand(std::function<void(const char *)> cb) { _cmdCb = cb; }
void XrouteWebSocket::onBuildStatus(std::function<void(StaticJsonDocument<512> &)> cb) { _statusCb = cb; }

void XrouteWebSocket::sendText(uint8_t clientId, const char *message)
{
  _ws.sendTXT(clientId, message);
}

void XrouteWebSocket::sendToAll(const char *message)
{
  _ws.broadcastTXT(message);
}

void XrouteWebSocket::broadcastStatus()
{
  if (!_statusCb)
    return;
  _doc.clear();
  _statusCb(_doc);
  size_t len = serializeJson(_doc, _outBuf, sizeof(_outBuf));
  _ws.broadcastTXT(_outBuf, len);
}

void XrouteWebSocket::start(UBaseType_t priority, BaseType_t core)
{
  // Initialize Wi-Fi AP+STA
  WiFi.mode(WIFI_AP_STA);
  if (_apSsid)
    WiFi.softAP(_apSsid, _apPass);
  if (_staSsid)
    WiFi.begin(_staSsid, _staPass);
  Serial.print("[XrouteWebSucket]:Trying to connect  to " + String(_staSsid));
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  // print my ip's
  Serial.print("[XrouteWebSucket]:AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[XrouteWebSucket]:STA IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize mDNS
  MDNS.begin(_host);
  MDNS.addService("ws", "tcp", _port);

  // Initialize WebSocket
  _ws.begin();
  //_ws = WebSocketsServer(_port);
  _ws.onEvent([this](uint8_t num, WStype_t type, uint8_t *payload, size_t length)
              { this->_handleEvent(num, type, payload, length); });

  // Launch loop task
  xTaskCreatePinnedToCore(
      _taskEntry,
      "XrouteWS",
      4096,
      this,
      priority,
      nullptr,
      core);
}

void XrouteWebSocket::_run()
{
  for (;;)
  {
    _ws.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void XrouteWebSocket::_handleEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
    if (_connectCb)
      _connectCb(num);
    if (_statusCb)
      broadcastStatus();
    break;
  case WStype_DISCONNECTED:
    if (_disconnectCb)
      _disconnectCb(num);
    break;
  case WStype_TEXT:
    if (_cmdCb)
    {
      char buf[length + 1];
      memcpy(buf, payload, length);
      buf[length] = '\0';
      _cmdCb(buf);
    }
    break;
  default:
    break;
  }
}