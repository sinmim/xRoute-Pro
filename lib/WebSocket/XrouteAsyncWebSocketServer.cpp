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

  if (mode == WIFI_MODE_STA)
  {
    /*
    set my wifi name to LCD
    wifiManager.begin(_staSsid, _staPass, "XroutePro-Todo");
    wifiManager.onConnected([&]()
                            {
                              Serial.println("🔌 WiFi Connected");
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
                              Serial.println("✔ WebSocket server listening on port : " + String(_port));
                              currentMode = WIFI_MODE_STA;
                              return true;
                              //
                            });
    wifiManager.onDisconnected([&](int reason)
                               {
                                 Serial.println("⚠️ WiFi Disconnected. Reason: " + String(reason));
                                 //ws.end();
                                 //
                               });
    */
    // old code
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
        //return false;
        break;
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
    Serial.println("✔ WebSocket server listening on port : " + String(_port));
    currentMode = WIFI_MODE_STA;
    return true;
  }
  else if (mode == WIFI_MODE_AP)
  {
    IPAddress local_IP(192, 168, 4, 1); // default AP IP
    IPAddress gateway(0, 0, 0, 0);      // signal no internet
    IPAddress subnet(255, 255, 255, 0);

    Serial.println("▶WIFI_MODE_AP | AP_NAME:" + String(_apSsid) + " | AP_PASS:" + String(_apPass));
    WiFi.mode(WIFI_MODE_AP);
    if (!WiFi.softAPConfig(local_IP, gateway, subnet))
    {
      Serial.println("✖ Failed to configure AP with 0.0.0.0 gateway");
      return false;
    }
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
  else if (mode == WIFI_MODE_APSTA)
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
/*
void XrouteAsyncWebSocketServer::streamFile(const char *filename, AsyncWebSocketClient *client, const char *jsonKey)
{
  if (!client)
  {
    Serial.println("[streamFile] Error: No client provided.");
    return;
  }

  File file = SPIFFS.open(filename, "r");
  if (!file)
  {
    String errorMsg = "{\"error\":\"Failed to open file: " + String(filename) + "\"}";
    client->text(errorMsg.c_str());
    Serial.println("[streamFile] " + errorMsg);
    return;
  }

  // 1. Send the JSON header as text
  String header = "{\"" + String(jsonKey) + "\":";
  client->text(header.c_str());

  // 2. Stream the file content in chunks AS TEXT
  const size_t bufferSize = 1400;
  uint8_t buffer[bufferSize];

  while (file.available())
  {
    size_t len = file.read(buffer, bufferSize);
    if (len > 0)
    {
      // *** THE KEY CHANGE IS HERE ***
      // Create a temporary String from the buffer and send it as text.
      String chunk(reinterpret_cast<char *>(buffer), len);
      client->text(chunk.c_str());
    }
  }
  file.close();

  // 3. Send the JSON footer as text
  client->text("}");
  Serial.printf("[streamFile] Successfully streamed '%s' (as text) to client %u\n", filename, client->id());
}
*/
void XrouteAsyncWebSocketServer::streamFile(const char *filename, AsyncWebSocketClient *client, const char *jsonKey)
{
  if (!client)
  {
    Serial.println("[streamFile] Error: No client provided.");
    return;
  }

  File file = SPIFFS.open(filename, "r");
  if (!file)
  {
    String errorMsg = "{\"error\":\"Failed to open file: " + String(filename) + "\"}";
    client->text(errorMsg);
    Serial.println("[streamFile] " + errorMsg);
    return;
  }

  size_t fileSize = file.size();
  String header = "{\"" + String(jsonKey) + "\":[";

  String response;
  // Pre-allocate memory for the String in one go for efficiency.
  response.reserve(header.length() + fileSize + 2);

  // 1. Add the header
  response += header;

  // 2. Read the file in chunks and append as text (THE CORRECT WAY)
  const size_t readBufferSize = 256; // A small, efficient buffer for reading
  char readBuffer[readBufferSize];
  while (file.available())
  {
    // Read a chunk of the file into the buffer
    size_t bytesRead = file.readBytes(readBuffer, readBufferSize);
    if (bytesRead > 0)
    {
      // Append the buffer's content to the main response String,
      // specifying the length to handle any binary data correctly.
      response += String(readBuffer, bytesRead);
    }
  }
  file.close();

  // 3. Add the footer
  response += "]}";

  // 4. Send the entire payload in a single, complete WebSocket message
  client->text(response);

  Serial.printf("[streamFile] Successfully sent '%s' as a single message of %u bytes to client %u\n", filename, response.length(), client->id());
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
      // Serial.println("[WS] 👤 Only one client connected (excluded) — skipping send");
      Serial.println("[WS] 👤 skipping send");
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
            if (updatingClient != nullptr && updatingClient == client) // preventing other cliants to curropt incomming update data
            {
              _updateCb((const char *)data, len);
              updateProgress += len;
              return;
            }
            else
            {
              Serial.println("Droping other incomming data : " + String(reinterpret_cast<char *>(data), len) + "\n");
              return;
            }
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
                // if _cmdCb is "ping" answer pong
                if (strcmp(cmd, "PING") == 0)
                {
                  // ❗WARNING: Never call client->text() directly here. Always use sendToClient()
                  // because this runs in core system thread, not a FreeRTOS task.
                  sendToClient("PONG", client);
                  // client.
                  //  WS_LOG("pong=>socket");
                  // Serial.printf("[WS_DEBUG] PONG client:%d\n", client->id());
                  return;
                }
                // command validation
                if (len > maxCmdPkgSize)
                {
                  Serial.println("Dropping : TOO BIG FOR COMMAND BUFFER!");
                  return;
                }
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
  for (;;)
  {
    if (self->_ws)
    {
      self->_ws->cleanupClients();
      static size_t last_n = 0;
      size_t n = self->_ws->count();
      if (n != last_n)
      {
        Serial.printf("[WS] cleanup→ %u clients\n", (unsigned)n);
        last_n = n;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  } }, "WS_Cleanup", 2048, this, 1, nullptr);

  // sending task
  if (_sendNotify == nullptr)
    _sendNotify = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore([](void *arg)
                          {
                            auto self = (XrouteAsyncWebSocketServer *)arg;
                            for (;;)
                            {
                              // ✅ Use self->_sendNotify here instead of raw _sendNotify
                              if (xSemaphoreTake(self->_sendNotify, pdMS_TO_TICKS(50)) == pdTRUE)
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
                                    /*
                                    The sendToAll function already acts as the "gatekeeper" to your queue. Once a message has been accepted into the queue,
                                    the WS_SEND_TASK should trust the underlying AsyncTCP library to handle the transmission and its own flow control.
                                    By removing the redundant second check, you ensure that any message you queue will be passed to the library for sending,
                                    preventing it from being dropped and breaking the vicious cycle.
                                    */
                                    // if (self->_ws->availableForWriteAll())
                                    //{
                                    // self->_ws->textAll(cmd.message.c_str());
                                    //}
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
                            }
                            //
                          },
                          "WS_SEND_TASK", 4096, this, 4, nullptr, 1);
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

/*
To-Do List for Future Improvements
Here are some key areas to look at later to further improve the robustness and efficiency of your project.

1. (High Priority) Investigate Memory Safety Margin (streamFile) while i did update ui by postman and then lcd ask the ui and it causes crash maybe i need to update index of the ui after a while
and also i should prevent creating ui send task in multiple requests and i think i did it already
The "one time crash" you mentioned suggests that the current "All-in-One" method, while functional, might be operating close to the device's memory limit.

Task: Quantify the memory usage and determine if the system is at risk with larger UI files.

Action Plan:

Temporarily add logging to the streamFile function to print the available heap before and after the response.reserve() call using ESP.getFreeHeap().

Test with the largest UI configuration file you expect to ever use. If the free heap drops to a very low level (e.g., less than 4-5 KB), the system is at risk of crashing again.

Contingency Plan: If memory proves to be too tight, implement the "start/end protocol" we discussed. It is the most memory-safe solution and guarantees stability regardless of file size, at the cost of requiring more logic on the client-side.

2. (Medium Priority) Stabilize the WiFi Connection
Our very first troubleshooting session was for a crash related to rapid WiFi connect/disconnect cycles. While we fixed the resulting WebSocket bug, the underlying cause of the unstable WiFi was never addressed.

Task: Investigate the root cause of WiFi disconnections (e.g., Reason: 8 - ASSOC_LEAVE, Reason: 15 - 4WAY_HANDSHAKE_TIMEOUT).

Action Plan:

Check the device's power supply for stability, as voltage drops can cause the radio to fail.

Evaluate the WiFi signal strength (RSSI) in its typical operating location.

Add more robust logging to your WiFi connection manager to track the frequency and reasons for disconnects over a longer period.

3. (Medium Priority) Optimize Task Stack Sizes
We identified that some tasks were allocated very large stacks (e.g., 10KB), which reserves a lot of RAM that the heap could otherwise use. Freeing this memory will make the entire system more stable.

Task: Right-size the stack for all major FreeRTOS tasks.

Action Plan:

For each long-running task in your setup() function, periodically log its "Stack High Water Mark" using uxTaskGetStackHighWaterMark(taskHandle). This tells you the minimum amount of free stack space the task has ever had.

Based on this data, safely reduce the stack sizes in your xTaskCreate calls to be closer to the actual usage (while still leaving a safe buffer of a few hundred bytes). This could free up many kilobytes of RAM for your heap.
*/