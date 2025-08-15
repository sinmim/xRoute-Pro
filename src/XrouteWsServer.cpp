#include "XrouteWsServer.h"
#include <map> // For per-client JSON buffers

// A static map to hold partial JSON data for each connecting client
static std::map<uint32_t, String> jsonBuffers;

XrouteWsServer::XrouteWsServer()
{
    _sendNotify = xSemaphoreCreateBinary();
}

XrouteWsServer::~XrouteWsServer()
{
    end();
    if (_sendNotify)
    {
        vSemaphoreDelete(_sendNotify);
        _sendNotify = nullptr;
    }
}

void XrouteWsServer::begin()
{
    if (_server)
    {
        // Already running
        return;
    }

    _server = new AsyncWebServer(port);
    _ws = new AsyncWebSocket("/");

    _ws->onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
                 {
                     this->handleEvent(server, client, type, arg, data, len);
                     //
                 });

    _server->addHandler(_ws);
    _server->begin();

    // Start the dedicated task for sending messages from the queue
    xTaskCreate(
        sendTask,
        "wsSendTask",
        4096,
        this,
        5, // High priority for network tasks
        &_sendTaskHandle);
    xTaskCreate(
        cleanupTask,
        "wsCleanupTask",
        2048,
        this,
        1, // Low priority cleanup task
        &_cleanupTaskHandle);
    Serial.println("[WS Server] WebSocket server started on port " + String(port));
}

void XrouteWsServer::end()
{
    if (_sendTaskHandle)
    {
        vTaskDelete(_sendTaskHandle);
        _sendTaskHandle = nullptr;
    }
    if (_server)
    {
        _ws->closeAll();
        _server->end();
        delete _server;
        _server = nullptr;
        _ws = nullptr; // _ws is owned by _server
        Serial.println("[WS Server] WebSocket server stopped.");
    }
}

// In XrouteWsServer.cpp
// REPLACE your handleEvent function with this one.

void XrouteWsServer::handleEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    uint32_t id = client->id();

    switch (type)
    {
    case WS_EVT_CONNECT:
    {
        // We check getClientCount() + 1 because the new client is not yet included in the count.
        // Or more simply, if current count is already at the max, reject the new one.
        if (getClientCount() >= _maxClients)
        {
            Serial.printf("[WS Server] Connection Rejected: Server is full (max clients: %u).\n", _maxClients);
            client->close();
            return; // Stop processing this connection event
        }
        // If we are not full, proceed with the normal connection logic
        client->setCloseClientOnQueueFull(false);
        Serial.printf("[WS Server][%u] Client connected. (Total: %u/%u)\n", id, getClientCount() + 1, _maxClients);
        if (_onConnectCb)
        {
            _onConnectCb(id);
        }
        break;
    }
    case WS_EVT_DISCONNECT:
        Serial.printf("[WS Server][%u] Client disconnected.\n", id);
        jsonBuffers.erase(id); // Clean up JSON buffer on disconnect
        if (_onDisconnectCb)
        {
            _onDisconnectCb(id);
        }
        // This gives the TCP stack a moment to clean up the closed socket
        // before another connection rapidly arrives, preventing resource exhaustion.
        vTaskDelay(pdMS_TO_TICKS(50));
        break;

    case WS_EVT_DATA:
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;

        if (_updatingFlg && id == _updatingClientId)
        {
            if (_onUpdateCb)
            {
                _onUpdateCb(id, data, len, _updateProgress, _updateLen);
                _updateProgress += len;
            }
            return;
        }

        if (info->opcode == WS_TEXT)
        {
            if (len > 0 && data[0] != '{')
            {
                char cmd[len + 1];
                memcpy(cmd, data, len);
                cmd[len] = '\0';

                if (strcmp(cmd, "PING") == 0)
                {
                    sendToClient(id, "PONG");
                    return;
                }

                if (_onCommandCb)
                {
                    _currentClientContext = id; // Set context RIGHT BEFORE callback
                    _onCommandCb(id, cmd);
                    _currentClientContext = 0; // Clear context RIGHT AFTER callback
                }
            }
            else
            {
                auto &buffer = jsonBuffers[id];
                buffer.concat((const char *)data, len);

                if (info->final)
                {
                    _doc.clear();
                    DeserializationError err = deserializeJson(_doc, buffer);

                    if (err == DeserializationError::Ok)
                    {
                        if (_onJsonCb)
                        {
                            _currentClientContext = id; // Set context RIGHT BEFORE callback
                            _onJsonCb(id, _doc);
                            _currentClientContext = 0; // Clear context RIGHT AFTER callback
                        }
                    }
                    else
                    {
                        Serial.printf("[WS Server][%u] JSON parse error: %s\n", id, err.c_str());
                    }
                    jsonBuffers.erase(id);
                }
            }
        }
        break;
    }

    case WS_EVT_ERROR:
        Serial.printf("[WS Server][%u] WebSocket error(%u): %s\n", id, *((uint16_t *)arg), (char *)data);
        break;

    default:
        break;
    }
}

// --- Public Method Implementations ---

void XrouteWsServer::onConnect(std::function<void(uint32_t)> cb) { _onConnectCb = cb; }
void XrouteWsServer::onDisconnect(std::function<void(uint32_t)> cb) { _onDisconnectCb = cb; }
void XrouteWsServer::onCommand(std::function<void(uint32_t, const char *)> cb) { _onCommandCb = cb; }
void XrouteWsServer::onJson(std::function<void(uint32_t, JsonDocument &)> cb) { _onJsonCb = cb; }
void XrouteWsServer::onUpdate(std::function<void(uint32_t, const uint8_t *, size_t, size_t, size_t)> cb) { _onUpdateCb = cb; }

size_t XrouteWsServer::getClientCount() const
{
    return _ws ? _ws->count() : 0;
}

void XrouteWsServer::sendToClient(uint32_t clientId, const char *message)
{
    if (!_server)
        return;
    WsServerQueueItem item;
    item.message = std::string(message);
    item.clientId = clientId;

    std::lock_guard<std::mutex> lock(_queueMutex);
    _sendQueue.push(item);
    xSemaphoreGive(_sendNotify);
}

void XrouteWsServer::sendToAll(const char *message)
{
    if (!_server)
        return;
    WsServerQueueItem item;
    item.message = std::string(message);
    item.clientId = 0; // 0 is the flag for broadcast

    std::lock_guard<std::mutex> lock(_queueMutex);
    _sendQueue.push(item);
    xSemaphoreGive(_sendNotify);
}

void XrouteWsServer::sendToAllExcept(const char *message, uint32_t excludedClientId)
{
    if (!_server)
        return;
    WsServerQueueItem item;
    item.message = std::string(message);
    item.clientId = 0;                       // Still a broadcast
    item.excludeClientId = excludedClientId; // But with an exclusion

    std::lock_guard<std::mutex> lock(_queueMutex);
    _sendQueue.push(item);
    xSemaphoreGive(_sendNotify);
}

void XrouteWsServer::cleanupTask(void *param)
{
    XrouteWsServer *self = static_cast<XrouteWsServer *>(param);
    Serial.println("[WS Cleanup] Task started.");
    for (;;)
    {
        if (self->_ws)
        {
            // This is the important call to the underlying library
            self->_ws->cleanupClients();
        }
        // Wait for 5 seconds before the next cleanup
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void XrouteWsServer::sendTask(void *param)
{
    XrouteWsServer *self = static_cast<XrouteWsServer *>(param);
    WsServerQueueItem item;

    for (;;)
    {
        if (xSemaphoreTake(self->_sendNotify, portMAX_DELAY) == pdTRUE)
        {
            while (true)
            {
                {
                    std::lock_guard<std::mutex> lock(self->_queueMutex);
                    if (self->_sendQueue.empty())
                    {
                        break; // Queue is empty, go back to waiting
                    }
                    item = self->_sendQueue.front();
                    self->_sendQueue.pop();
                }

                if (!self->_ws)
                    continue;

                if (item.clientId == 0) // Broadcast
                {
                    if (item.excludeClientId == 0) // To All
                    {
                        self->_ws->textAll(item.message.c_str());
                    }
                    else // To All Except One
                    {
                        // FINAL CORRECTION: Remove 'const'. We need a mutable reference.
                        auto &clients = self->_ws->getClients();
                        for (auto &client : clients)
                        {
                            if (client.status() == WS_CONNECTED && client.id() != item.excludeClientId)
                            {
                                // This will now work because 'client' is not const
                                client.text(item.message.c_str());
                            }
                        }
                    }
                }
                else // Send to a specific client
                {
                    self->_ws->text(item.clientId, item.message.c_str());
                }
            }
        }
    }
}

// --- Firmware Update Methods ---

void XrouteWsServer::startUpdate(uint32_t clientId, uint32_t totalLength)
{
    if (_updatingFlg)
    {
        Serial.println("[WS Server] Warning: Another update is already in progress.");
        return;
    }
    _updatingClientId = clientId;
    _updateLen = totalLength;
    _updateProgress = 0;
    _updatingFlg = true;
    Serial.printf("[WS Server] Starting update for client %u, size: %u bytes.\n", clientId, totalLength);
}

void XrouteWsServer::endUpdate()
{
    _updatingFlg = false;
    _updatingClientId = 0;
    Serial.println("[WS Server] Update process finished.");
}

bool XrouteWsServer::isUpdating() const
{
    return _updatingFlg;
}

float XrouteWsServer::getUpdateProgress() const
{
    if (_updateLen == 0)
        return 0.0f;
    return (float)_updateProgress / (float)_updateLen;
}