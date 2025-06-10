#include "XrouteAsyncWebSocketServer.h"
#include <ESPmDNS.h>
#define ME Serial.print("[XrouteAsyncWebSocketServer]:")

enum WiFiMode
{
  MODE_STA,
  MODE_AP,
  MODE_AP_STA
};
wifi_mode_t XrouteAsyncWebSocketServer ::currentMode = WIFI_MODE_STA;

void XrouteAsyncWebSocketServer::setSTA(const char *ssid, const char *pass)
{
  _staSsid = ssid;
  _staPass = pass;
}

void XrouteAsyncWebSocketServer::printWiFiDetails()
{
  if (currentMode == MODE_STA && WiFi.status() == WL_CONNECTED)
  {
    Serial.print("STA IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("STA RSSI: ");
    Serial.println(WiFi.RSSI());
  }
  else if (currentMode == MODE_AP)
  {
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("AP SSID: ");
    Serial.println(_apSsid);
    Serial.print("AP Clients: ");
    Serial.println(WiFi.softAPgetStationNum());
  }
}

<<<<<<< HEAD

=======
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
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
<<<<<<< HEAD
  //WiFi.setAutoConnect(false);//now deprecated 
=======
  // WiFi.setAutoConnect(false);//now deprecated
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
  WiFi.setAutoReconnect(true);
  delay(100);
  // 1) set the mode

  if (mode == WIFI_MODE_APSTA)
  {
    WiFi.mode(WIFI_MODE_APSTA);
    // 2) start the AP
    WiFi.softAP(_apSsid, _apPass);
    Serial.println("▶ Soft-AP started");
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
    currentMode = WIFI_MODE_STA;
    return true;
  }
  else if (mode == WIFI_MODE_AP)
  {
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(_apSsid, _apPass);
    Serial.println("▶ Soft-AP started");
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
    currentMode = WIFI_MODE_AP;
    return true;
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

void XrouteAsyncWebSocketServer::setStatusBuilder(std::function<void(StaticJsonDocument<4096> &)> cb)
{
  _statusCb = cb;
}

void XrouteAsyncWebSocketServer::sendToAll(const char *message)
{
  if (_ws)
    _ws->textAll(message);
}

void XrouteAsyncWebSocketServer::sendToClient(const char *message, AsyncWebSocketClient *client)
{
  if (client)
    client->text(message);
}

void XrouteAsyncWebSocketServer::sendToThisClient(const char *message)
{
  if (LastClient)
    LastClient->text(message);
}

void XrouteAsyncWebSocketServer::SendToAllExcludeClient(const char *message, AsyncWebSocketClient *ExClient)
{
  std::lock_guard<std::mutex> lock(_clientsLock);
  for (AsyncWebSocketClient *c : _clients)
  {
<<<<<<< HEAD
    TickType_t currentLoopDelay = apModeBaseCycleDelay; // Default delay for this iteration

    if (XrouteAsyncWebSocketServer::currentMode == MODE_AP)
    {
      uint8_t connectedClients = WiFi.softAPgetStationNum();

      if (connectedClients > 0)
      {
        Serial.printf("[apMonitorTask AP Mode]: %u client(s) connected to AP. STA scan deferred.\n", connectedClients);
        // If clients are connected, we significantly delay or skip the scan
        // to maintain AP stability. The task will sleep for apModeBaseCycleDelay.
      }
      else if (!apScanInProgress)
      {
        // No clients connected AND no scan in progress, consider starting a scan.
        if (self->_staSsid && strlen(self->_staSsid) > 0)
        {
          Serial.print("[apMonitorTask AP Mode]: No clients. Attempting targeted ASYNC scan for SSID: '");
          Serial.print(self->_staSsid);
          Serial.println("'");

          int16_t scanCommandStatus = WiFi.scanNetworks(
              true,           // async
              true,           // show_hidden
              false,          // passive (active is faster for targeted)
              120,            // max_ms_per_chan
              0,              // channel (all)
              self->_staSsid, // specific ssid
              NULL            // bssid
          );

          if (scanCommandStatus == WIFI_SCAN_RUNNING)
          {
            Serial.println("[apMonitorTask AP Mode]: Targeted ASYNC scan initiated.");
            apScanInProgress = true;
            currentLoopDelay = apModeScanCheckInterval; // Check scan completion sooner
          }
          else
          {
            Serial.printf("[apMonitorTask AP Mode]: Failed to start scan, status: %d\n", scanCommandStatus);
            apScanInProgress = false;
            // Wait for the base cycle before trying again
          }
        }
        else
        {
          Serial.println("[apMonitorTask AP Mode]: No STA SSID configured, scan skipped.");
        }
      }
      else
      { // apScanInProgress is true, check for completion
        // Serial.println("[apMonitorTask AP Mode]: Checking ASYNC scan completion..."); // Can be verbose
        int16_t scanResultCount = WiFi.scanComplete();

        if (scanResultCount == WIFI_SCAN_RUNNING)
        {
          // Serial.println("[apMonitorTask AP Mode]: ASYNC scan still running...");
          currentLoopDelay = apModeScanCheckInterval; // Continue checking frequently
        }
        else if (scanResultCount == WIFI_SCAN_FAILED)
        {
          Serial.println("[apMonitorTask AP Mode]: ASYNC scan failed.");
          WiFi.scanDelete();
          apScanInProgress = false;
        }
        else if (scanResultCount >= 0)
        {
          Serial.printf("[apMonitorTask AP Mode]: ASYNC scan FINISHED. Networks matching SSID '%s' found: %d\n", self->_staSsid, scanResultCount);
          bool targetNetworkFound = (scanResultCount > 0);

          if (targetNetworkFound)
          {
            Serial.print("[apMonitorTask AP Mode]: Target STA network '");
            Serial.print(self->_staSsid);
            Serial.println("' found! Attempting to switch.");
            //self->switchToSTAMode();
            self->init(WIFI_MODE_STA); 
            // After switchToSTAMode, mode will change, loop will re-evaluate.
            // If switch is successful, we won't be in MODE_AP.
            // If switch fails and returns to MODE_AP, apScanInProgress is false.
          }
          else
          {
            Serial.println("[apMonitorTask AP Mode]: Target STA network not found.");
          }
          WiFi.scanDelete();
          apScanInProgress = false;
        }
      } // end of !apScanInProgress / else
      vTaskDelay(currentLoopDelay); // Use the determined delay for this AP mode cycle
    }
    else if (XrouteAsyncWebSocketServer::currentMode == MODE_AP_STA)
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        if (self->_staSsid && strlen(self->_staSsid) > 0)
        {
          Serial.println("[apMonitorTask AP_STA Mode]: STA disconnected. Attempting reconnect...");
          WiFi.disconnect(false); // Attempt to disconnect STA only
          delay(100);
          WiFi.begin(self->_staSsid, self->_staPass);
        }
      }
      else
      {
        // Serial.println("[apMonitorTask AP_STA Mode]: STA connected. AP active.");
      }
      vTaskDelay(pdMS_TO_TICKS(20000)); // Check STA status every 20s
    }
    else if (XrouteAsyncWebSocketServer::currentMode == MODE_STA)
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("[apMonitorTask STA Mode]: STA Disconnected! This task doesn't currently auto-switch from pure STA.");
        // Your main loop or Xroute class might handle this scenario by calling startAPMode() etc.
      }
      else
      {
        // Serial.println("[apMonitorTask STA Mode]: Connected. Monitoring for mode switch triggers suspended by this task.");
      }
      vTaskDelay(pdMS_TO_TICKS(30000));
    }
    else
    {
      Serial.printf("[apMonitorTask]: In unknown mode (%d), idling.\n", XrouteAsyncWebSocketServer::currentMode);
      vTaskDelay(pdMS_TO_TICKS(30000));
    }
  } // End of main task loop
}

<<<<<<< HEAD
=======
/*
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
void XrouteAsyncWebSocketServer::registerEvents()
{
  _ws->onEvent([this](AsyncWebSocket *socketServer, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
               {
                 switch (type)
                 {
                 case WS_EVT_CONNECT:
<<<<<<< HEAD
                   Serial.printf("[WS][%u] Client CONNECTED from %s\n", client->id(), client->remoteIP().toString().c_str());
                   // You could also call a user-defined onConnect callback here if you add one
                   break;

                 case WS_EVT_DISCONNECT:
                   Serial.printf("[WS][%u] Client DISCONNECTED\n", client->id());
                   // You could also call a user-defined onDisconnect callback here
=======
                   _clientCount++;
                   Serial.printf("WS[%u] connected (now %u clients)\n",
                                 client->id(), (unsigned)_clientCount.load());

                   break;

                 case WS_EVT_DISCONNECT:
                   _clientCount--;
                   Serial.printf("WS[%u] disconnected (now %u clients)\n",
                                 client->id(), (unsigned)_clientCount.load());
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
                   break;

                 case WS_EVT_PONG:
                   Serial.printf("[WS][%u] Pong received\n", client->id());
                   break;

                 case WS_EVT_ERROR:
                   Serial.printf("[WS][%u] ERROR(%u): %s\n", client->id(), *((uint16_t *)arg), (char *)data);
                   break;

                 case WS_EVT_DATA:
                 {
                   AwsFrameInfo *info = (AwsFrameInfo *)arg;
                   if (info->final && info->opcode == WS_TEXT && info->index == 0)
                   {                                // Process complete text messages
                     String msg((char *)data, len); // Construct String with specific length

<<<<<<< HEAD
                     //Serial.print("[WS_EVT_DATA] Received Full TEXT Message: '"); // For debugging
                     //Serial.print(msg);
                     //Serial.println("'");
=======
                     // Serial.print("[WS_EVT_DATA] Received Full TEXT Message: '"); // For debugging
                     // Serial.print(msg);
                     // Serial.println("'");
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
                     if (msg.startsWith("{"))
                     {               // Basic check for JSON
                       _doc.clear(); // _doc is a class member StaticJsonDocument
                       DeserializationError error = deserializeJson(_doc, msg);
                       if (error == DeserializationError::Ok)
                       {
                         Serial.println("[WS_EVT_DATA] JSON Parsed OK. Calling _jsonCb.");
                         _jsonCb(_doc); // Call the user's JSON callback
                       }
                       else
                       {
                         Serial.print("[WS_EVT_DATA] ⚠ JSON parse failed: ");
                         Serial.println(error.c_str());
                       }
                     }
                     else if (_cmdCb)
                     {
<<<<<<< HEAD
                       //Serial.println("[WS_EVT_DATA] Not JSON or no JSON callback. Calling _cmdCb.");
=======
                       // Serial.println("[WS_EVT_DATA] Not JSON or no JSON callback. Calling _cmdCb.");
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
                       _cmdCb(msg.c_str()); // Call the user's command callback
                     }
                     else
                     {
                       Serial.println("[WS_EVT_DATA] No suitable callback registered for the received message.");
                     }
                   }
                   else if (info->opcode != WS_TEXT)
                   {
                     Serial.printf("[WS_EVT_DATA] Received NON-TEXT frame (opcode: %u). Length: %u. Ignoring.\n", info->opcode, len);
                   }
                   else if (!info->final)
                   {
                     // ESPAsyncWebServer should handle fragmentation and deliver complete messages.
                     // If you see this, it might be part of a larger message being assembled.
                     Serial.printf("[WS_EVT_DATA] Received a fragmented frame (len: %u, index: %u).\n", len, info->index);
                   }
                   break;
                 } // End of WS_EVT_DATA

                 default:
                   // Serial.printf("[WS] Unhandled event type: %d\n", type); // Optional: for debugging other event types
                   break;
                 } // End of switch (type)
               }); // End of _ws->onEvent lambda
}

<<<<<<< HEAD
=======
*/
=======
    if (!c)
      continue; // just in case
    if (c == ExClient)
      continue; // skip that one
    c->text(message);
  }
}

>>>>>>> 5ea013e (Websocket OTA task added and timeout mechanisem works nice)
// — in XrouteAsyncWebSocketServer.cpp:
#include <map>
static std::map<uint32_t, String> jsonBuffers;

void XrouteAsyncWebSocketServer::registerEvents()
{
  _ws->onEvent([this](AsyncWebSocket *server,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len)
               {
    switch (type) {
      case WS_EVT_CONNECT: {
         // Add to our registry
                {
          std::lock_guard<std::mutex> lock(_clientsLock);
          _clients.push_back(client);
        }
         int cid = client->id();
        _clientCount++;
        Serial.printf("[WS][%u] CONNECTED  (total %u)\n",
                      cid, (unsigned)_clientCount.load());
        break;
      }

      case WS_EVT_DISCONNECT: {
                // Remove from registry
        {
          std::lock_guard<std::mutex> lock(_clientsLock);
          auto it = std::find(_clients.begin(), _clients.end(), client);
          if (it != _clients.end()) {
            _clients.erase(it);
          }
        }
        uint32_t cid = client->id();
        _clientCount--;
        Serial.printf("[WS][%u] DISCONNECTED  (total %u)\n",
                      cid, (unsigned)_clientCount.load());
        break;
      }

case WS_EVT_DATA: {
  if(updatingFlg)
  {
    _updateCb((const char*)data,len);
    updateProgress += len;
    return;
  }
  auto* info = reinterpret_cast<AwsFrameInfo*>(arg);

  // only consider text / continuation frames
  if ((info->opcode == WS_TEXT || info->opcode == WS_CONTINUATION) && data && len) {

    // 1) Plain-text “commands” come as a single WS_TEXT frame, index==0,
    //    and don’t start with '{'
    if (info->opcode == WS_TEXT 
        && info->index == 0 
        && data[0] != '{') {
      // capture last client for sendToThisClient()
      LastClient = client;

      // copy into a null-terminated buffer
      char cmd[len+1];
      memcpy(cmd, data, len);
      cmd[len] = '\0';

      // invoke your command callback
      if (_cmdCb) {
        _cmdCb(cmd);
      }
    }
    // 2) Otherwise, treat as JSON (possibly fragmented)
    else {
      auto& buf = jsonBuffers[client->id()];
      buf.concat(reinterpret_cast<const char*>(data), len);

      // try parsing on every chunk
      _doc.clear();
      DeserializationError err = deserializeJson(_doc, buf);

      if (!err) {
        // full JSON received
        if (_jsonCb) {
          _jsonCb(_doc);
        }
        jsonBuffers.erase(client->id());
      }
      else if (err == DeserializationError::IncompleteInput) {
        // waiting for more fragments…
      }
      else {
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
    } });
}

<<<<<<< HEAD
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
void XrouteAsyncWebSocketServer::begin()
=======
void XrouteAsyncWebSocketServer::begin(wifi_mode_t mode)
>>>>>>> 5ea013e (Websocket OTA task added and timeout mechanisem works nice)
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======

>>>>>>> 5ea013e (Websocket OTA task added and timeout mechanisem works nice)
  xTaskCreate([](void *arg)
              {
  auto self = (XrouteAsyncWebSocketServer*)arg;
  for(;;) {
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
    vTaskDelay(pdMS_TO_TICKS(1000));  // every 5 seconds
  } }, "WS_Cleanup", 2048, this, 1, nullptr);
>>>>>>> daf7903 (Socket cliant cleanup and led indicator Added)
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