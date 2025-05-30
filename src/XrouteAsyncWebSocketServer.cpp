#include "XrouteAsyncWebSocketServer.h"
#include <ESPmDNS.h>
#define ME Serial.print("[XrouteAsyncWebSocketServer]:")

enum WiFiMode
{
  MODE_STA,
  MODE_AP,
  MODE_AP_STA
};
int XrouteAsyncWebSocketServer ::currentMode = MODE_STA;

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
  //WiFi.setAutoConnect(false);//now deprecated 
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
    Serial.printf("▶ Connecting to %s …", _staSsid);
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

void XrouteAsyncWebSocketServer::onJson(std::function<void(StaticJsonDocument<512> &)> cb)
{
  _jsonCb = cb;
}

void XrouteAsyncWebSocketServer::setStatusBuilder(std::function<void(StaticJsonDocument<512> &)> cb)
{
  _statusCb = cb;
}

void XrouteAsyncWebSocketServer::sendToAll(const char *message)
{
  if (_ws)
    _ws->textAll(message);
}

void XrouteAsyncWebSocketServer::apMonitorTask(void *pv)
{
  auto *self = static_cast<XrouteAsyncWebSocketServer *>(pv);
  Serial.println("[apMonitorTask]: Task started.");

  static bool apScanInProgress = false; // Keeps track of an ongoing asynchronous scan
  // Define intervals for AP mode scanning
  const TickType_t apModeBaseCycleDelay = pdMS_TO_TICKS(30000);   // Base cycle: check to scan every 30 seconds
  const TickType_t apModeScanCheckInterval = pdMS_TO_TICKS(1000); // How often to call scanComplete if scan is running

  for (;;)
  {
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

void XrouteAsyncWebSocketServer::registerEvents()
{
  _ws->onEvent([this](AsyncWebSocket *socketServer, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
               {
                 switch (type)
                 {
                 case WS_EVT_CONNECT:
                   Serial.printf("[WS][%u] Client CONNECTED from %s\n", client->id(), client->remoteIP().toString().c_str());
                   // You could also call a user-defined onConnect callback here if you add one
                   break;

                 case WS_EVT_DISCONNECT:
                   Serial.printf("[WS][%u] Client DISCONNECTED\n", client->id());
                   // You could also call a user-defined onDisconnect callback here
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

                     //Serial.print("[WS_EVT_DATA] Received Full TEXT Message: '"); // For debugging
                     //Serial.print(msg);
                     //Serial.println("'");
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
                       //Serial.println("[WS_EVT_DATA] Not JSON or no JSON callback. Calling _cmdCb.");
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

void XrouteAsyncWebSocketServer::begin()
{
  if (!init(WIFI_MODE_APSTA))
  {
    init(WIFI_MODE_AP);
  }
}

#undef ME