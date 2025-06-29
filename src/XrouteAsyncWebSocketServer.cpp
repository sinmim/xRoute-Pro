#include "XrouteAsyncWebSocketServer.h"
#include <ESPmDNS.h>
#include "freertos/semphr.h"
#define ME Serial.print("[XrouteAsyncWebSocketServer]:")

wifi_mode_t XrouteAsyncWebSocketServer ::currentMode = WIFI_MODE_STA;

void XrouteAsyncWebSocketServer::setSTA(const char *ssid, const char *pass)
{
  _staSsid = ssid;
  _staPass = pass;
}

void XrouteAsyncWebSocketServer::printWiFiDetails()
{
  if (currentMode == WIFI_MODE_STA && WiFi.status() == WL_CONNECTED)
  {
    Serial.print("STA IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("STA RSSI: ");
    Serial.println(WiFi.RSSI());
  }
  else if (currentMode == WIFI_MODE_AP)
  {
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("AP SSID: ");
    Serial.println(_apSsid);
    Serial.print("AP Clients: ");
    Serial.println(WiFi.softAPgetStationNum());
  }
}

bool XrouteAsyncWebSocketServer::init(wifi_mode_t mode)
{
  int timeout = 10000;
  // first lets deinit all the states befor so we could start clean
  if (_server)
  {
    _server->end();
    delete _server;
    _server = nullptr;
  }
  if (_ws)
  {
    _ws->cleanupClients();
    delete _ws;
    _ws = nullptr;
  }
  MDNS.end();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // WiFi.setAutoConnect(false);//now deprecated
  WiFi.setAutoReconnect(true);
  delay(100);
  // 1) set the mode

  if (mode == WIFI_MODE_APSTA)
  {

    WiFi.mode(WIFI_MODE_APSTA);
    // 2) start the AP
    WiFi.softAP(_apSsid, _apPass);
    Serial.println("▶WIFI_MODE_APSTA");
    // 3) start the STA
    Serial.printf("▶ Connecting to %s\n", _staSsid);
    Serial.printf("▶ Connecting to %s\n", _staPass);

    WiFi.begin(_staSsid, _staPass);
    // start a timer
    unsigned long currentMillis = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
      if (millis() - currentMillis > timeout)
      {
        ME;
        Serial.println("Time oute");
        return false;
      }
      delay(500);
      Serial.print('.');
    }
    Serial.println();
    Serial.print("✔ STA IP: ");
    Serial.println(WiFi.localIP());

    // 4) mDNS responder
    if (!MDNS.begin(_host))
    {
      Serial.println("✖ mDNS init failed");
      return false;
    }
    else
    {
      MDNS.addService("ws", "tcp", _port);
      Serial.println("✔ mDNS responder started:" + String(_host) + ".local");
    }
    // 5) WebSocket setup
    _ws = new AsyncWebSocket("/");
    registerEvents();
    _server = new AsyncWebServer(_port);
    _server->addHandler(_ws);
    _server->begin();
    Serial.println("✔ WebSocket server listening on port 81");
    currentMode = WIFI_MODE_APSTA;
    return true;
  }
  else if (mode == WIFI_MODE_STA)
  {
    Serial.println("▶WIFI_MODE_STA");
    Serial.println("✔ Connecting to SSID: " + String(_staSsid) + "  PASS: " + String(_staPass));
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(_staSsid, _staPass);
    unsigned long currentMillis = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
      if (millis() - currentMillis > timeout)
      {
        ME;
        Serial.println("Time oute");
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      Serial.print('.');
    }
    Serial.println();
    Serial.print("✔ WIFI_MODE_STA IP: ");
    Serial.println(WiFi.localIP());

    // 4) mDNS responder
    if (!MDNS.begin(_host))
    {
      Serial.println("✖ mDNS init failed");
      return false;
    }
    else
    {
      MDNS.addService("ws", "tcp", _port);
      Serial.println("✔ mDNS responder started:" + String(_host) + ".local");
    }
    // 5) WebSocket setup
    _ws = new AsyncWebSocket("/");
    registerEvents();
    _server = new AsyncWebServer(_port);
    _server->addHandler(_ws);
    _server->begin();
    Serial.println("✔ WebSocket server listening on port 81");
    currentMode = WIFI_MODE_STA;
    return true;
  }
  else if (mode == WIFI_MODE_AP)
  {
    Serial.println("▶WIFI_MODE_AP | AP_NAME:" + String(_apSsid) + " | AP_PASS:" + String(_apPass));

    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(_apSsid, _apPass);
    if (!MDNS.begin(_host))
    {
      Serial.println("✖ mDNS init failed");
      return false;
    }
    else
    {
      MDNS.addService("ws", "tcp", _port);
      Serial.println("✔ mDNS responder started:" + String(_host) + ".local");
    }
    // 5) WebSocket setup
    _ws = new AsyncWebSocket("/");
    registerEvents();
    _server = new AsyncWebServer(_port);
    _server->addHandler(_ws);
    _server->begin();
    Serial.println("✔ WebSocket server listening on port 81");
    currentMode = WIFI_MODE_AP;
    return true;
  }
  else
  {
    Serial.println("✖ Invalid mode");
    return false;
  }
}

void XrouteAsyncWebSocketServer::setAP(const char *ssid, const char *pass)
{
  _apSsid = ssid;
  _apPass = pass;
}

void XrouteAsyncWebSocketServer::setHostname(const char *name)
{
  _host = name;
}

void XrouteAsyncWebSocketServer::setPort(uint16_t wsPort)
{
  _port = wsPort;
}

void XrouteAsyncWebSocketServer::onCommand(std::function<void(const char *)> cb)
{
  _cmdCb = cb;
}

void XrouteAsyncWebSocketServer::onJson(std::function<void(StaticJsonDocument<4096> &)> cb)
{
  _jsonCb = cb;
}

void XrouteAsyncWebSocketServer::sendToAll(const char *message)
{
  if (!_ws)
  {
    // Serial.println("[WS] ❌ WebSocket server not initialized");
    return;
  }

  if (!_ws->availableForWriteAll())
  {
    Serial.println("[WS] ❌ Cannot broadcast — send queues full for all clients");
    return;
  }

  QueuedWebSocketCmd cmd;
  cmd.message = std::string(message);
  cmd.who = ALL_CLIENTS;

  {
    std::lock_guard<std::mutex> lock(_sendQueueLock);
    _wsSendQueue.push(cmd);
  }

  if (_sendNotify)
    xSemaphoreGive(_sendNotify);
}

void XrouteAsyncWebSocketServer::sendToClient(const char *message, AsyncWebSocketClient *client)
{
  if (!client)
    return;

  if (!_ws->availableForWrite(client->id()))
  {
    Serial.printf("[WS] ❌ Cannot respond to client %u — send queue full, response dropped\n", client->id());
    return;
  }

  QueuedWebSocketCmd cmd;
  cmd.message = std::string(message);
  cmd.client = client;
  cmd.who = THIS_CLIENT;

  {
    std::lock_guard<std::mutex> lock(_sendQueueLock);
    _wsSendQueue.push(cmd);
  }

  if (_sendNotify)
    xSemaphoreGive(_sendNotify);
}

void XrouteAsyncWebSocketServer::sendToThisClient(const char *message)
{
  if (!LastClient)
    return;

  if (!_ws->availableForWrite(LastClient->id()))
  {
    Serial.printf("[WS] ❌ Cannot respond to client %u — send queue full, response dropped\n", LastClient->id());
    return;
  }

  QueuedWebSocketCmd cmd;
  cmd.message = std::string(message);
  cmd.client = LastClient;
  cmd.who = THIS_CLIENT;

  {
    std::lock_guard<std::mutex> lock(_sendQueueLock);
    _wsSendQueue.push(cmd);
  }

  if (_sendNotify)
    xSemaphoreGive(_sendNotify);
}

void XrouteAsyncWebSocketServer::SendToAllExcludeClient(const char *message, AsyncWebSocketClient *exClient)
{
  if (!exClient)
    return;

  {
    std::lock_guard<std::mutex> lock(_clientsLock);

    if (_clients.size() <= 1)
    {
      // Only one client connected — it's the one to be excluded
      Serial.println("[WS] 👤 Only one client connected (excluded) — skipping send");
      return;
    }
  }

  QueuedWebSocketCmd cmd;
  cmd.message = std::string(message);
  cmd.client = exClient;
  cmd.who = EXCLUDE_CLIENT;

  {
    std::lock_guard<std::mutex> lock(_sendQueueLock);
    _wsSendQueue.push(cmd);
  }

  if (_sendNotify)
    xSemaphoreGive(_sendNotify);
}

// — in XrouteAsyncWebSocketServer.cpp:
#include <map>
static std::map<uint32_t, String> jsonBuffers;

void XrouteAsyncWebSocketServer::registerEvents()
{
  _ws->onEvent(
      [this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
      {
        switch (type)
        {
        case WS_EVT_CONNECT:
        {
          client->setCloseClientOnQueueFull(false); // saman : it will drop excessive datas to prevent client disconnection
          //  Add to our registry
          std::lock_guard<std::mutex> lock(_clientsLock);
          _clients.push_back(client);
          int cid = client->id();
          _clientCount++;
          Serial.printf("[WS][%u] CONNECTED  (total %u)\n", cid, (unsigned)_clientCount.load());
          break;
        }
        case WS_EVT_DISCONNECT:
        {
          // Remove from registry
          std::lock_guard<std::mutex> lock(_clientsLock);
          auto it = std::find(_clients.begin(), _clients.end(), client);
          if (it != _clients.end())
          {
            _clients.erase(it);
          }
          uint32_t cid = client->id();
          _clientCount--;
          Serial.printf("[WS][%u] DISCONNECTED  (total %u)\n", cid, (unsigned)_clientCount.load());
          break;
        }
        case WS_EVT_DATA:
        {
          if (updatingFlg)
          {
            _updateCb((const char *)data, len);
            updateProgress += len;
            return;
          }
          auto *info = reinterpret_cast<AwsFrameInfo *>(arg);
          // only consider text / continuation frames
          if ((info->opcode == WS_TEXT || info->opcode == WS_CONTINUATION) && data && len)
          {
            // 1) Plain-text “commands” come as a single WS_TEXT frame, index==0,
            //    and don’t start with '{'
            if (info->opcode == WS_TEXT && info->index == 0 && data[0] != '{')
            {
              // capture last client for sendToThisClient()
              LastClient = client;
              // copy into a null-terminated buffer
              char cmd[len + 1];
              memcpy(cmd, data, len);
              cmd[len] = '\0';
              // invoke your command callback
              if (_cmdCb)
              {
                _cmdCb(cmd);
              }
            }
            // 2) Otherwise, treat as JSON (possibly fragmented)
            else
            {
              auto &buf = jsonBuffers[client->id()];
              buf.concat(reinterpret_cast<const char *>(data), len);
              // try parsing on every chunk
              _doc.clear();
              DeserializationError err = deserializeJson(_doc, buf);
              if (!err)
              {
                // full JSON received
                if (_jsonCb)
                {
                  _jsonCb(_doc);
                }
                jsonBuffers.erase(client->id());
              }
              else if (err == DeserializationError::IncompleteInput)
              {
                // waiting for more fragments…
              }
              else
              {
                // fatal parse error—log and drop buffer
                Serial.printf("JSON parse error: %s\n", err.c_str());
                jsonBuffers.erase(client->id());
              }
            }
          }
          break;
        }
        default:
          break;
        }
      });
}

void XrouteAsyncWebSocketServer::begin(wifi_mode_t mode)
{
  if (mode == WIFI_MODE_APSTA)
  {
    if (!init(WIFI_MODE_APSTA))
    {
      init(WIFI_MODE_AP);
    }
  }
  if (mode == WIFI_MODE_STA)
  {
    init(WIFI_MODE_STA);
  }
  if (mode == WIFI_MODE_AP)
  {
    init(WIFI_MODE_AP);
  }
  // Cleanup task
  xTaskCreate([](void *arg)
              {
  auto self = (XrouteAsyncWebSocketServer*)arg;
  for(;;) 
  {
    //if wifi is connected
    if (WiFi.status() == WL_CONNECTED)
    {
      // this forces the library to sweep out any TCP sockets
      // that silently went away
      self->_ws->cleanupClients();
      // now _ws->count() reflects the real # of active clients
      static size_t last_n=0;
      size_t n = self->_ws->count();
      if (n!=last_n)
      {
        Serial.printf("[WS] cleanup→ %u clients\n", (unsigned)n);
      }
      last_n = n;
    }
    else
    {
        //Serial.println("[WS] Wifi connection not available");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));  // every 5 seconds
  } }, "WS_Cleanup", 2048, this, 1, nullptr);

  // sending task
  if (_sendNotify == nullptr)
    _sendNotify = xSemaphoreCreateBinary();

  xTaskCreate([](void *arg)
              {
    auto self = (XrouteAsyncWebSocketServer*)arg;
    for (;;)
    {
        // ✅ Use self->_sendNotify here instead of raw _sendNotify
        if (xSemaphoreTake(self->_sendNotify, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            while (true)
            {
                QueuedWebSocketCmd cmd;
                {
                    std::lock_guard<std::mutex> lock(self->_sendQueueLock);
                    if (self->_wsSendQueue.empty())
                        break;
                    cmd = self->_wsSendQueue.front();
                    self->_wsSendQueue.pop();
                }
                if (cmd.who == THIS_CLIENT && cmd.client)
                {
                    if (self->_ws->availableForWrite(cmd.client->id()))
                        cmd.client->text(cmd.message.c_str());
                }
                else if (cmd.who == ALL_CLIENTS)
                {
                    if (self->_ws->availableForWriteAll())
                        self->_ws->textAll(cmd.message.c_str());
                }
                else if (cmd.who == EXCLUDE_CLIENT && cmd.client)
                {
                    std::lock_guard<std::mutex> lock(self->_clientsLock);
                    for (AsyncWebSocketClient *c : self->_clients)
                    {
                        if (c && c != cmd.client && self->_ws->availableForWrite(c->id()))
                            c->text(cmd.message.c_str());
                    }
                }
            }
        }
    } }, "WS_SEND_TASK", 4096, this, 1, nullptr);
}

wifi_mode_t XrouteAsyncWebSocketServer::switchMode(wifi_mode_t mode)
{
  begin(mode);
  return currentMode;
}

wifi_mode_t XrouteAsyncWebSocketServer::getMode()
{
  return currentMode;
}

#undef ME