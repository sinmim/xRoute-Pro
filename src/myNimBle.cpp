#include "myNimBle.h"
#include <string>

// Static member definitions
std::vector<whiteListInfo> MyBle::whiteList;
std::vector<waiteListInfo> MyBle::waiteList;
bool MyBle::connectedFlg = false;
bool MyBle::newConnectionMade = false;
NimBLEAddress *MyBle::pServerAddress = nullptr;
String MyBle::bleAddTmp = "";

// UUIDs
static const NimBLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");

// Constructor: Initialize RTOS objects and tasks
MyBle::MyBle(bool clientMode) : isClientMode(clientMode),
                                pClient(nullptr),
                                pServer(nullptr),
                                pRemoteCharacteristic(nullptr),
                                pServerCharacteristic(nullptr)
{
    sendQueueMutex = xSemaphoreCreateMutex();
    listMutex = xSemaphoreCreateRecursiveMutex();
    bleWriteQueue = xQueueCreate(10, sizeof(BleWriteRequest));

    if (sendQueueMutex == NULL || listMutex == NULL || bleWriteQueue == NULL)
    {
        NIMBLE_LOG("!!! Error creating RTOS objects !!!");
    }

    xTaskCreate(sendTask, "SendTask", 4096, this, 1, NULL);
    xTaskCreate(timeOutTask, "TimeOutTask", 4096, this, 1, NULL);
    xTaskCreate(bleWriteTask, "BleWriteTask", 4096, this, 2, NULL);
}

// Destructor: Clean up RTOS objects
MyBle::~MyBle()
{
    vSemaphoreDelete(sendQueueMutex);
    vSemaphoreDelete(listMutex);
    vQueueDelete(bleWriteQueue);
    if (pServerAddress != nullptr)
    {
        delete pServerAddress;
        pServerAddress = nullptr;
    }
}

// Client Begin
void MyBle::begin(std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb)
{
    clientCallBack = cb;
    if (isClientMode)
    {
        NimBLEDevice::init("LabobinxSmart");

        if (!this->pClient)
        {
            this->pClient = NimBLEDevice::createClient();
            if (!this->pClient)
            {
                NIMBLE_LOG("!!! Failed to create NimBLE client !!!");
                return;
            }
            this->pClient->setClientCallbacks(new MyClientCallback(this), true);
            NIMBLE_LOG("BLE Client Initialized");
        }
        else
        {
            NIMBLE_LOG("BLE Client already exists.");
        }
    }
    else
    {
        NIMBLE_LOG("Error: Called begin() in Server Mode.");
    }
}

// Server Begin
void MyBle::beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> cb)
{
    serverCallBack = cb;

    if (isClientMode)
    {
        NIMBLE_LOG("Error: Called beginServer() in Client Mode.");
        return;
    }

    if (this->pServer)
    {
        NIMBLE_LOG("BLE Server already exists.");
        NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
        if (pAdvertising && !pAdvertising->isAdvertising() &&
            this->pServer->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
        {
            pAdvertising->start();
            NIMBLE_LOG("Restarting Advertising...");
        }
        return;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char uniqueID[7];
    snprintf(uniqueID, sizeof(uniqueID), "-%02X%02X", mac[4], mac[5]);
    std::string deviceName = "xRoutePro";
    NimBLEDevice::init(deviceName);
    deviceName += uniqueID;
    deviceCode = uniqueID;
    NIMBLE_LOG("BLE Server Initializing with name: %s", deviceName.c_str());

    NimBLEDevice::setSecurityAuth(false, false, false);
    this->pServer = NimBLEDevice::createServer();
    this->pServer->setCallbacks(new MyServerCallbacks(this), true);

    NimBLEService *pService = this->pServer->createService(serviceUUID);
    pServerCharacteristic = pService->createCharacteristic(
        charUUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::NOTIFY);
    pServerCharacteristic->setCallbacks(new MyCallbacks(*this));
    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(deviceName);
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->enableScanResponse(true);
    if (pAdvertising->start())
    {
        NIMBLE_LOG("BLE Server Advertising started successfully!");
    }
    else
    {
        NIMBLE_LOG("!!! Failed to start BLE Advertising !!!");
    }
}

NimBLECharacteristic* MyBle::getServerCharacteristic() const
{
    if (!isClientMode)
    {
        return pServerCharacteristic; // Return the member pointer
    }
    return nullptr; // Return null if in client mode or not initialized
}
// --- TASKS ---

void MyBle::sendTask(void *param)
{
    MyBle *self = static_cast<MyBle *>(param);
    std::vector<uint16_t> recipient_handles;

    for (;;)
    {
        String payload;
        if (xSemaphoreTake(self->sendQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (!self->sendQueue.empty())
            {
                payload = self->sendQueue.front();
                self->sendQueue.pop();
            }
            xSemaphoreGive(self->sendQueueMutex);
        }

        if (payload.length() > 0 && self->isConnected() && self->pServerCharacteristic)
        {
            recipient_handles.clear();

            xSemaphoreTakeRecursive(self->listMutex, portMAX_DELAY);
            for (const auto &client : self->whiteList) {
                recipient_handles.push_back(client.connInfo.getConnHandle());
            }
            xSemaphoreGiveRecursive(self->listMutex);

            if(!recipient_handles.empty()) {
                self->pServerCharacteristic->setValue(payload);
                for(uint16_t handle : recipient_handles) {
                    self->pServerCharacteristic->notify(handle);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void MyBle::timeOutTask(void *param)
{
    MyBle *self = static_cast<MyBle *>(param);
    std::vector<std::pair<NimBLEAddress, uint16_t>> to_disconnect;
    std::vector<std::pair<NimBLEAddress, uint16_t>> to_prompt;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        to_disconnect.clear();
        to_prompt.clear();

        xSemaphoreTakeRecursive(self->listMutex, portMAX_DELAY);

        for (auto it = self->waiteList.begin(); it != self->waiteList.end(); ++it)
        {
            int timeout = it->clientResponsing ? WAITE_FOR_USER_ENTERING_PASSKEY : AUT_TIME_OUT;
            if (millis() - it->startMilis > timeout)
            {
                to_disconnect.push_back({it->MacAdd, it->connInfo.getConnHandle()});
                continue;
            }

            if (!it->clientResponsing && it->retyForRespons < 5 && millis() - it->startMilis > (it->retyForRespons + 1) * 1000)
            {
                 to_prompt.push_back({it->MacAdd, it->connInfo.getConnHandle()});
                 it->retyForRespons++;
            }
        }
        xSemaphoreGiveRecursive(self->listMutex);

        for (const auto& p : to_prompt) {
            self->sendStringToMac("GIVE_ME_PASSKEY\n", p.second);
            NIMBLE_LOG("Sent 'GIVE_ME_PASSKEY' to %s", p.first.toString().c_str());
        }

        for (const auto& p : to_disconnect) {
            self->sendStringToMac("AUT_TIMEOUT\n", p.second);
            if (self->pServer) self->pServer->disconnect(p.second);
        }
    }
}

void MyBle::bleWriteTask(void *param)
{
    MyBle *self = static_cast<MyBle *>(param);
    BleWriteRequest request;

    for (;;)
    {
        if (xQueueReceive(self->bleWriteQueue, &request, portMAX_DELAY) == pdTRUE)
        {
            bool is_whitelisted = self->isInWhiteList(request.address);

            if (is_whitelisted)
            {
                if (self->serverCallBack && self->pServer)
                {
                    // FIXED: Get the valid NimBLEConnInfo object from the server
                    NimBLEConnInfo info = self->pServer->getPeerInfo(request.connHandle);
                    uint8_t* pData = reinterpret_cast<uint8_t*>(const_cast<char*>(request.data.c_str()));
                    self->serverCallBack(self->pServerCharacteristic, info, pData, request.data.length());
                }
            }
            else
            {
                self->authenticate(request.address, request.connHandle, request.data);
            }
        }
    }
}


// --- Core Logic & Methods ---

void MyBle::sendStringToMac(String str, uint16_t connHandle)
{
    if (!isConnected() || !pServerCharacteristic) return;
    pServerCharacteristic->setValue(str);
    pServerCharacteristic->notify(connHandle);
}

void MyBle::clearwhitelist()
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    whiteList.clear();
    xSemaphoreGiveRecursive(listMutex);
}

void MyBle::clearwaitelist()
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    waiteList.clear();
    xSemaphoreGiveRecursive(listMutex);
}

void MyBle::disconnectAllWhiteList()
{
    std::vector<uint16_t> handles_to_disconnect;
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    for (const auto &client : whiteList)
    {
        handles_to_disconnect.push_back(client.connInfo.getConnHandle());
    }
    xSemaphoreGiveRecursive(listMutex);

    if (pServer) {
        for(uint16_t handle : handles_to_disconnect) {
            pServer->disconnect(handle);
        }
    }
}

void MyBle::justSend(String data)
{
    if (this->pServerCharacteristic)
    {
        this->pServerCharacteristic->setValue(data);
        this->pServerCharacteristic->notify();
    }
}

bool MyBle::connectToServer(NimBLEAddress pAddress)
{
    if (!isClientMode || !this->pClient) return false;
    if (this->pClient->isConnected())
    {
        if (this->pClient->getPeerAddress().equals(pAddress)) return true;
        this->pClient->disconnect();
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    NIMBLE_LOG("Connecting to Address: %s", pAddress.toString().c_str());

    if (this->pClient->connect(pAddress))
    {
        NIMBLE_LOG("Connected successfully!");
        NimBLERemoteService *pRemoteService = this->pClient->getService(serviceUUID);
        if (!pRemoteService) { this->pClient->disconnect(); return false; }
        
        this->pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
        if (!this->pRemoteCharacteristic) { this->pClient->disconnect(); return false; }

        if (this->pRemoteCharacteristic->canNotify())
        {
            auto notifyCallbackWrapper =
                [this](NimBLERemoteCharacteristic *pChr, uint8_t *pData, size_t length, bool isNotify)
            {
                if (this->clientCallBack) this->clientCallBack(pChr, pData, length, isNotify);
            };

            if (this->pRemoteCharacteristic->subscribe(true, notifyCallbackWrapper))
            {
                connectedFlg = true;
                return true;
            }
        }
    }
    this->pClient->disconnect();
    this->pRemoteCharacteristic = nullptr;
    return false;
}

bool MyBle::connectToMac(String macAddress)
{
    // FIXED: Added the second argument to the constructor
    NimBLEAddress address(std::string(macAddress.c_str()), BLE_ADDR_PUBLIC);
    return connectToServer(address);
}

void MyBle::sendString(String str)
{
    if (!isConnected()) return;
    if (isClientMode)
    {
        if (this->pRemoteCharacteristic && this->pRemoteCharacteristic->canWrite())
        {
            this->pRemoteCharacteristic->writeValue(str, false);
        }
    }
    else // Server mode
    {
        if (xSemaphoreTake(sendQueueMutex, portMAX_DELAY) == pdTRUE)
        {
            sendQueue.push(str);
            xSemaphoreGive(sendQueueMutex);
        }
    }
}

void MyBle::sendData(const char *data) { sendString(String(data)); }

void MyBle::sendLongString(String str)
{
    return; // Intentionally disabled as in original code
    const int CHUNKSIZE = 200;
    while (str.length() > 0)
    {
        String strChunk = str.substring(0, CHUNKSIZE);
        str = str.substring(CHUNKSIZE);
        sendString(strChunk);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void MyBle::disconnect()
{
    if (isClientMode && this->pClient && this->pClient->isConnected())
    {
        this->pClient->disconnect();
    }
    else if (!isClientMode && this->pServer && this->pServer->getConnectedCount() > 0)
    {
        disconnectAllWhiteList();
    }
}

bool MyBle::isConnected()
{
    if (isClientMode && this->pClient)
    {
        connectedFlg = this->pClient->isConnected();
    }
    else if (!isClientMode && this->pServer)
    {
        connectedFlg = (this->pServer->getConnectedCount() > 0);
    }
    else
    {
        connectedFlg = false;
    }
    return connectedFlg;
}

void MyBle::pause() { runFlg = false; }
void MyBle::resume() { runFlg = true; }
bool MyBle::isRunning() { return runFlg; }
bool MyBle::isNewConnection()
{
    if (newConnectionMade)
    {
        newConnectionMade = false;
        return true;
    }
    return false;
}

bool MyBle::isSendQueueBusy()
{
    if (isClientMode) return false;
    bool isBusy = false;
    if (xSemaphoreTake(sendQueueMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        isBusy = !sendQueue.empty();
        xSemaphoreGive(sendQueueMutex);
    } else {
        isBusy = true;
    }
    return isBusy;
}

String MyBle::directRead()
{
    if (!directReadingFlg) return "";
    TickType_t startTick = xTaskGetTickCount();
    while (!dataReady)
    {
        if (xTaskGetTickCount() - startTick > pdMS_TO_TICKS(5000)) return "";
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    dataReady = false;
    return strBLE;
}

void MyBle::deleteAllBonds()
{
    NIMBLE_LOG("Deleting all bonds...");
    int numBonds = NimBLEDevice::getNumBonds();
    Serial.printf("Found %d bonded devices.\n", numBonds);
    
    if (numBonds > 0)
    {
        // FIXED: Reverted to original, correct logic
        NimBLEAddress *pAddrs = new NimBLEAddress[numBonds];
        for (int i = 0; i < numBonds; i++)
        {
            pAddrs[i] = NimBLEDevice::getBondedAddress(i);
        }

        for (int i = 0; i < numBonds; i++)
        {
            NIMBLE_LOG("Deleting bond for: %s", pAddrs[i].toString().c_str());
            NimBLEDevice::deleteBond(pAddrs[i]);
        }
        delete[] pAddrs;
    }
}

void MyBle::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
    if (advertisedDevice->isAdvertisingService(serviceUUID))
    {
        NIMBLE_LOG("Found target service on device: %s", advertisedDevice->getAddress().toString().c_str());
        bleAddTmp = advertisedDevice->getAddress().toString().c_str();
        if (NimBLEDevice::getScan()->isScanning())
        {
            NimBLEDevice::getScan()->stop();
        }
    }
}

void MyBle::setPassKey(uint32_t _password, bool wipe)
{
    while (isSendQueueBusy())
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    passKey = _password;
    NimBLEDevice::setSecurityPasskey(_password);
    if (wipe)
    {
        deleteAllBonds();
        disconnectAllWhiteList();
        clearwhitelist();
        clearwaitelist();
    }
    NIMBLE_LOG("BLE PASSKEY set to: %u", passKey);
}

uint32_t MyBle::getPassKey() const
{
    return passKey;
}

// --- List Management (Thread-Safe) ---

void MyBle::addToWhiteList(const NimBLEAddress &address, const NimBLEConnInfo &connInfo)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    for (auto &client : whiteList)
    {
        if (client.MacAdd.equals(address))
        {
            client.connInfo = connInfo;
            xSemaphoreGiveRecursive(listMutex);
            return;
        }
    }
    if (whiteList.size() < WHITE_LIST_SIZE)
    {
        whiteList.push_back({address, connInfo});
    }
    xSemaphoreGiveRecursive(listMutex);
}

void MyBle::addToWaiteList(const NimBLEAddress &address, const NimBLEConnInfo &connInfo)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    for (auto &client : waiteList)
    {
        if (client.MacAdd.equals(address))
        {
            client.connInfo = connInfo;
            client.startMilis = millis();
            client.passwordRetryCount = 0;
            client.retyForRespons = 0;
            client.clientResponsing = false;
            client.accumulatedData.clear();
            xSemaphoreGiveRecursive(listMutex);
            return;
        }
    }
    if (waiteList.size() < WAITE_LIST_SIZE)
    {
        waiteList.push_back({address, connInfo, millis(), 0, 0, false, ""});
    }
    xSemaphoreGiveRecursive(listMutex);
}

bool MyBle::isInWhiteList(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    bool found = false;
    for (const auto &client : whiteList)
    {
        if (client.MacAdd.equals(address))
        {
            found = true;
            break;
        }
    }
    xSemaphoreGiveRecursive(listMutex);
    return found;
}

bool MyBle::isInWaiteList(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    bool found = false;
    for (const auto &client : waiteList)
    {
        if (client.MacAdd.equals(address))
        {
            found = true;
            break;
        }
    }
    xSemaphoreGiveRecursive(listMutex);
    return found;
}

void MyBle::removeFromWaiteList(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    waiteList.erase(std::remove_if(waiteList.begin(), waiteList.end(),
        [&](const waiteListInfo& client) {
            return client.MacAdd.equals(address);
        }), waiteList.end());
    xSemaphoreGiveRecursive(listMutex);
}

void MyBle::removeFromWhiteList(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    whiteList.erase(std::remove_if(whiteList.begin(), whiteList.end(),
        [&](const whiteListInfo& client) {
            return client.MacAdd.equals(address);
        }), whiteList.end());
    xSemaphoreGiveRecursive(listMutex);
}

int MyBle::getWaiteLisindex(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    for (int i = 0; i < waiteList.size(); i++)
    {
        if (waiteList[i].MacAdd.equals(address))
        {
            xSemaphoreGiveRecursive(listMutex);
            return i;
        }
    }
    xSemaphoreGiveRecursive(listMutex);
    return -1;
}

int MyBle::getWhiteLisindex(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    for (int i = 0; i < whiteList.size(); i++)
    {
        if (whiteList[i].MacAdd.equals(address))
        {
            xSemaphoreGiveRecursive(listMutex);
            return i;
        }
    }
    xSemaphoreGiveRecursive(listMutex);
    return -1;
}

void MyBle::setResponsFromClient(const NimBLEAddress &address, bool userIsEnteringPass)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    for (auto &client : waiteList) {
        if (client.MacAdd.equals(address)) {
            client.clientResponsing = userIsEnteringPass;
            break;
        }
    }
    xSemaphoreGiveRecursive(listMutex);
}

bool MyBle::getResponsFromClient(const NimBLEAddress &address)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
    bool responding = false;
    for (auto &client : waiteList) {
        if (client.MacAdd.equals(address)) {
            responding = client.clientResponsing;
            break;
        }
    }
    xSemaphoreGiveRecursive(listMutex);
    return responding;
}

void MyBle::authenticate(const NimBLEAddress &address, uint16_t connHandle, const std::string &data)
{
    xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);

    int index = -1;
    for(int i=0; i<waiteList.size(); ++i) {
        if(waiteList[i].MacAdd.equals(address)) {
            index = i;
            break;
        }
    }

    if (index == -1)
    {
        xSemaphoreGiveRecursive(listMutex);
        return;
    }

    waiteList[index].accumulatedData += data;
    std::string& accumulatedData = waiteList[index].accumulatedData;

    int endPos;
    while ((endPos = accumulatedData.find('\n')) != std::string::npos)
    {
        std::string command_str = accumulatedData.substr(0, endPos);
        String command = command_str.c_str();
        accumulatedData.erase(0, endPos + 1);

        if (command.startsWith("PASSKEY="))
        {
            uint32_t _passKey = command.substring(8).toInt();
            if (_passKey == passKey)
            {
                NimBLEConnInfo connInfo = waiteList[index].connInfo;
                addToWhiteList(address, connInfo);
                removeFromWaiteList(address);
                
                xSemaphoreGiveRecursive(listMutex);
                sendStringToMac("PASSKEY_ACCEPTED\n", connHandle);
                NIMBLE_LOG("PASSKEY correct. %s added to whitelist.", address.toString().c_str());
                return;
            }
            else
            {
                waiteList[index].passwordRetryCount++;
                int retries = waiteList[index].passwordRetryCount;
                
                xSemaphoreGiveRecursive(listMutex);
                sendStringToMac("INVALID_PASSWORD\n", connHandle);
                
                if (retries > MAX_RETRY_ATTEMPTS)
                {
                    if (pServer) pServer->disconnect(connHandle);
                    return;
                }
                xSemaphoreTakeRecursive(listMutex, portMAX_DELAY);
            }
        }
        else if (command.startsWith("WAITING_FOR_PASSKEY"))
        {
            setResponsFromClient(address, true);
        }
    }
    xSemaphoreGiveRecursive(listMutex);
}


// --- Callback Class Implementations ---

void MyBle::MyClientCallback::onConnect(NimBLEClient *pclient)
{
    newConnectionMade = true;
}

void MyBle::MyClientCallback::onDisconnect(NimBLEClient *pclient, int reason)
{
    connectedFlg = false;
    if (pMyBleInstance)
    {
        pMyBleInstance->pRemoteCharacteristic = nullptr;
    }
}

void MyBle::MyServerCallbacks::onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
{
    const NimBLEAddress addr = connInfo.getAddress();
    NIMBLE_LOG("Device connected: %s", addr.toString().c_str());

    if (!pMyBleInstance->isInWhiteList(addr))
    {
        pMyBleInstance->addToWaiteList(addr, connInfo);
    }
    
    connectedFlg = true;
    newConnectionMade = true;
    if (pServer->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
    {
        NimBLEDevice::startAdvertising();
    }
}

void MyBle::MyServerCallbacks::onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason)
{
    NIMBLE_LOG("Device disconnected: %s, Reason: %d", connInfo.getAddress().toString().c_str(), reason);
    
    pMyBleInstance->removeFromWaiteList(connInfo.getAddress());
    pMyBleInstance->removeFromWhiteList(connInfo.getAddress());

    connectedFlg = (pServer->getConnectedCount() > 0);
    if (pServer->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
    {
        NimBLEDevice::startAdvertising();
    }
}

void MyBle::MyServerCallbacks::onAuthenticationComplete(NimBLEConnInfo &connInfo)
{
    NIMBLE_LOG("Authentication complete for %s. Encrypted: %s",
                connInfo.getAddress().toString().c_str(),
                connInfo.isEncrypted() ? "Yes" : "No");
}

uint32_t MyBle::MyServerCallbacks::onPassKeyRequest()
{
    return 123456;
}

bool MyBle::MyServerCallbacks::onConfirmPIN(uint32_t pass_key)
{
    return true;
}

void MyBle::MyCallbacks::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    NimBLEAttValue value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    if (parent.directReadingFlg)
    {
        parent.strBLE = value.c_str();
        parent.dataReady = true;
        return;
    }

    BleWriteRequest request;
    request.address = connInfo.getAddress();
    request.connHandle = connInfo.getConnHandle();
    request.data = std::string(reinterpret_cast<const char*>(value.data()), value.length());

    if (xQueueSend(parent.bleWriteQueue, &request, 0) != pdTRUE)
    {
        NIMBLE_LOG("!!! BLE write queue full, dropping data from %s !!!", connInfo.getAddress().toString().c_str());
    }
}

void MyBle::MyCallbacks::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    NIMBLE_LOG("Read request from %s", connInfo.getAddress().toString().c_str());
}

void MyBle::MyCallbacks::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
{
    NIMBLE_LOG("Subscribe event from %s", connInfo.getAddress().toString().c_str());
}