#ifndef XROUTE_WS_SERVER_H
#define XROUTE_WS_SERVER_H

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <functional>
#include <queue>
#include <mutex>
#include <atomic>
#include <vector>

// Forward declaration to avoid including the whole header
class AsyncWebSocketClient;

// Structure for the thread-safe message queue
struct WsServerQueueItem
{
    std::string message;
    uint32_t clientId = 0;        // 0 means broadcast to all
    uint32_t excludeClientId = 0; // For broadcasting to all except one
};

class XrouteWsServer
{
public:
    XrouteWsServer();
    ~XrouteWsServer();

    void begin();
    void end();
    void setPort(uint16_t port) { this->port = port; }
    void setMaxClients(size_t limit) { _maxClients = limit; } // ADD THIS LINE

    // --- Client & State Management ---
    size_t getClientCount() const;
    void sendToAll(const char *message);
    void sendToClient(uint32_t clientId, const char *message);
    void sendToAllExcept(const char *message, uint32_t excludedClientId);
    uint32_t getClient() { return _currentClientContext; }

    // --- Firmware Update Management ---
    void startUpdate(uint32_t clientId, uint32_t totalLength);
    void endUpdate();
    float getUpdateProgress() const;
    bool isUpdating() const;

    // --- Event Callbacks ---
    void onConnect(std::function<void(uint32_t clientId)> cb);
    void onDisconnect(std::function<void(uint32_t clientId)> cb);
    void onCommand(std::function<void(uint32_t clientId, const char *cmd)> cb);
    void onJson(std::function<void(uint32_t clientId, JsonDocument &doc)> cb);
    void onUpdate(std::function<void(uint32_t clientId, const uint8_t *data, size_t len, size_t index, size_t total)> cb);

private:
    uint16_t port = 81;
    size_t _maxClients = 10;
    uint32_t _currentClientContext = 0;
    // Async server objects
    AsyncWebServer *_server = nullptr;
    AsyncWebSocket *_ws = nullptr;

    // Callbacks
    std::function<void(uint32_t)> _onConnectCb = nullptr;
    std::function<void(uint32_t)> _onDisconnectCb = nullptr;
    std::function<void(uint32_t, const char *)> _onCommandCb = nullptr;
    std::function<void(uint32_t, JsonDocument &)> _onJsonCb = nullptr;
    std::function<void(uint32_t, const uint8_t *, size_t, size_t, size_t)> _onUpdateCb = nullptr;

    // Internal event handler
    void handleEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

    // Thread-safe sending queue
    std::queue<WsServerQueueItem> _sendQueue;
    std::mutex _queueMutex;
    SemaphoreHandle_t _sendNotify = nullptr;
    TaskHandle_t _sendTaskHandle = nullptr;
    TaskHandle_t _cleanupTaskHandle = nullptr;
    static void sendTask(void *param);
    static void cleanupTask(void *param);

    // Update state
    std::atomic<bool> _updatingFlg{false};
    uint32_t _updateLen = 0;
    uint32_t _updateProgress = 0;
    uint32_t _updatingClientId = 0;

    // Per-client JSON buffer
    StaticJsonDocument<4096> _doc;
};

#endif