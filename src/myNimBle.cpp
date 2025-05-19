#include "myNimBle.h"
#include <string> // Include for std::string

std::vector<whiteListInfo> MyBle::whiteList;
std::vector<waiteListInfo> MyBle::waiteList;

// Define static members
bool MyBle::connectedFlg = false;
bool MyBle::newConnectionMade = false;
NimBLEAddress *MyBle::pServerAddress = nullptr;
String MyBle::bleAddTmp = "";

// UUIDs (const is good practice)
static const NimBLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");

// Constructor: Initialize member pointers
MyBle::MyBle(bool clientMode) : isClientMode(clientMode),
                                pClient(nullptr),
                                pServer(nullptr),
                                pRemoteCharacteristic(nullptr), // Init non-static member
                                pServerCharacteristic(nullptr)  // Init non-static member
{
    sendQueueMutex = xSemaphoreCreateMutex();
    if (sendQueueMutex == NULL)
    {
        Serial.println("!!! Error creating send queue mutex !!!");
    }
    xTaskCreate(sendTask, "SendTask", 4096, this, 1, NULL);
    xTaskCreate(timeOutTask, "timeOutTask", 4096, this, 1, NULL);
}

// Destructor
MyBle::~MyBle()
{
    vSemaphoreDelete(sendQueueMutex);
    if (pServerAddress != nullptr)
    { // Check static address pointer
        delete pServerAddress;
        pServerAddress = nullptr;
    }
    // Consider deleting pClient/pServer if dynamically allocated?
    // NimBLEDevice::deinit() handles global cleanup if needed.
}

// Client Begin
void MyBle::begin(std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb)
{
    clientCallBack = cb;
    if (isClientMode)
    {
        NimBLEDevice::init("LabobinxSmart");

        if (!this->pClient)
        { // Use 'this->' for clarity accessing member
            this->pClient = NimBLEDevice::createClient();
            if (!this->pClient)
            {
                Serial.println("!!! Failed to create NimBLE client !!!");
                return;
            }
            this->pClient->setClientCallbacks(new MyClientCallback(this), true);
            Serial.println("BLE Client Initialized");
        }
        else
        {
            Serial.println("BLE Client already exists.");
        }
    }
    else
    {
        Serial.println("Error: Called begin() in Server Mode.");
    }
}

void MyBle::beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> cb)
{
    serverCallBack = cb;

    if (isClientMode)
    {
        Serial.println("Error: Called beginServer() in Client Mode.");
        return;
    }

    if (this->pServer)
    {
        Serial.println("BLE Server already exists.");
        NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
        if (pAdvertising && !pAdvertising->isAdvertising() &&
            this->pServer->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
        {
            pAdvertising->start();
            Serial.println("Restarting Advertising...");
        }
        return;
    }

    // Initialize device name with unique ID
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char uniqueID[7];
    snprintf(uniqueID, sizeof(uniqueID), "-%02X%02X", mac[4], mac[5]);
    std::string deviceName = "xRoutePro";
    NimBLEDevice::init(deviceName);
    deviceName += uniqueID;
    Serial.printf("BLE Server Initializing with name: %s\n", deviceName.c_str());

    // Disable security for open connections
    NimBLEDevice::setSecurityAuth(false, false, false);

    // Create server and set callbacks
    this->pServer = NimBLEDevice::createServer();
    this->pServer->setCallbacks(new MyServerCallbacks(this), true);

    // Create service and characteristic (no encryption)
    NimBLEService *pService = this->pServer->createService(serviceUUID);
    pServerCharacteristic = pService->createCharacteristic(
        charUUID,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);
    pServerCharacteristic->setCallbacks(new MyCallbacks(*this));
    pService->start();

    // Start advertising
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(deviceName);
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->enableScanResponse(true);

    if (pAdvertising->start())
    {
        Serial.println("BLE Server Advertising started successfully!");
    }
    else
    {
        Serial.println("!!! Failed to start BLE Advertising !!!");
    }
}

/*
void MyBle::beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> cb)
{
    serverCallBack = cb; // User callback needs updated signature!
    if (!isClientMode)
    {
        if (!this->pServer)
        { // Check member pointer
            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            char uniqueID[7];
            snprintf(uniqueID, sizeof(uniqueID), "-%02X%02X", mac[4], mac[5]);

            std::string deviceName = "xRoutePro";
            NimBLEDevice::init(deviceName);
            deviceName += uniqueID;
            Serial.printf("BLE Server Initializing with name: %s\n", deviceName.c_str());

            // Proper security initialization
            NimBLEDevice::setSecurityAuth(true, true, true);
            NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
            NimBLEDevice::setSecurityPasskey(123456);
            NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

            // Create Server instance
            this->pServer = NimBLEDevice::createServer();
            if (!this->pServer)
            {
                Serial.println("!!! Failed to create NimBLE server !!!");
                return;
            }
            this->pServer->setCallbacks(new MyServerCallbacks(this), true);

            // Create BLE Service
            NimBLEService *pService = this->pServer->createService(serviceUUID);
            if (!pService)
            {
                Serial.println("!!! Failed to create NimBLE service !!!");
                return;
            }

            // Create Characteristic with encryption properties
            // --- characteristic creation ---
            pServerCharacteristic = pService->createCharacteristic(
                charUUID,
                NIMBLE_PROPERTY::READ |
                    NIMBLE_PROPERTY::WRITE |
                    NIMBLE_PROPERTY::NOTIFY | // keep
                    NIMBLE_PROPERTY::READ_ENC |
                    NIMBLE_PROPERTY::WRITE_ENC); // <-- drop INDICATE
            if (!this->pServerCharacteristic)
            {
                Serial.println("!!! Failed to create NimBLE characteristic !!!");
                return;
            }

            this->pServerCharacteristic->setCallbacks(new MyCallbacks(*this));
            pService->start();

            // Setup advertising correctly
            NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
            if (pAdvertising)
            {
                pAdvertising->setName(deviceName);
                pAdvertising->addServiceUUID(serviceUUID);
                pAdvertising->enableScanResponse(true);

                if (pAdvertising->start())
                {
                    Serial.println("BLE Server Advertising started successfully!");
                }
                else
                {
                    Serial.println("!!! Failed to start BLE Advertising !!!");
                }
            }
            else
            {
                Serial.println("!!! Failed to get Advertising object !!!");
            }
        }
        else
        {
            Serial.println("BLE Server already exists.");
            // Restart advertising if not already advertising
            NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
            if (pAdvertising && !pAdvertising->isAdvertising() &&
                this->pServer->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
            {
                pAdvertising->start();
                Serial.println("Restarting Advertising...");
            }
        }
    }
    else
    {
        Serial.println("Error: Called beginServer() in Client Mode.");
    }
}

*/

// --- sendTask ---
void MyBle::sendTask(void *param)
{
    MyBle *self = static_cast<MyBle *>(param);
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

        if (payload.length() && self->isConnected() && self->pServerCharacteristic)
        {
            // send to white list clients only use foreach
            for (const auto &client : self->whiteList)
            {
                self->pServerCharacteristic->setValue(payload);
                self->pServerCharacteristic->notify(client.connInfo.getConnHandle());
            }
            vTaskDelay(pdMS_TO_TICKS(5)); // gentle pacing
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(20)); // idle sleep
        }
    }
}
//--- timeoutTask ---
void MyBle::timeOutTask(void *param)
{
    MyBle *self = static_cast<MyBle *>(param);

    for (;;)
    {
        for (int i = 0; i < waiteList.size(); i++)
        {
            // if user did not respond yet send him "GiveMePassKey\n" every 1000ms
            if (millis() - waiteList[i].startMilis > 100 &&                                  // waite for 100ms
                millis() - waiteList[i].startMilis > (waiteList[i].retyForRespons * 1000) && // every retry adds 1000ms
                waiteList[i].retyForRespons < 5 &&                                           // 5 retrys
                !waiteList[i].clientResponsing)                                              // no respons yet
            {
                if (self->pServer)
                {
                    self->sendStringToMac("GIVE_ME_PASSKEY\n", waiteList[i].connInfo);
                    waiteList[i].retyForRespons++;
                    Serial.printf("Sent 'GIVE_ME_PASSKEY' to %s\n", waiteList[i].MacAdd.toString().c_str());
                }
            }
            // calculate timeout in a single line
            int timeout = waiteList[i].clientResponsing ? WAITE_FOR_USER_ENTERING_PASSKEY : AUT_TIME_OUT;

            if (millis() - waiteList[i].startMilis > timeout)
            {
                if (self->pServer)
                {
                    // send cleint "timeout"
                    self->sendStringToMac("AUT_TIMEOUT\n", waiteList[i].connInfo);
                    self->pServer->disconnect(waiteList[i].connInfo.getConnHandle());
                    self->removeFromWaiteList(waiteList[i].MacAdd);
                    Serial.printf("Device %s timed out and disconnected\n", waiteList[i].MacAdd.toString().c_str());
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void MyBle::sendStringToMac(String str, NimBLEConnInfo &connInfo)
{
    if (!isConnected() || !pServerCharacteristic)
    {
        Serial.println("Not connected or characteristic not available.");
        return;
    }
    pServerCharacteristic->setValue(str);
    pServerCharacteristic->notify(connInfo.getConnHandle());
}

void MyBle::clearwhitelist()
{
    whiteList.clear();
}
void MyBle::clearwaitelist()
{
    waiteList.clear();
}

void MyBle::disconnectAllWhiteList()
{
    for (const auto &client : whiteList)
    {
        if (pServer)
        {
            pServer->disconnect(client.connInfo.getConnHandle());
        }
    }
}

void MyBle::justSend(String data)
{
    int i = 0;
    this->pServerCharacteristic->setValue(data);
    this->pServerCharacteristic->notify();
}

// Connect to Server (Client Mode)
bool MyBle::connectToServer(NimBLEAddress pAddress)
{
    if (!isClientMode || !this->pClient)
    { /*...*/
        return false;
    } // Use member pClient
    if (this->pClient->isConnected())
    {
        if (this->pClient->getPeerAddress().equals(pAddress))
            return true;
        else
        {
            this->pClient->disconnect();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    Serial.print("Connecting to Address: ");
    Serial.println(pAddress.toString().c_str());

    if (this->pClient->connect(pAddress))
    { // Use member pClient
        Serial.println("Connected successfully!");
        NimBLERemoteService *pRemoteService = this->pClient->getService(serviceUUID);
        if (!pRemoteService)
        { /*...*/
            this->pClient->disconnect();
            return false;
        }
        Serial.println("Found Service");

        // Assign to member pRemoteCharacteristic
        this->pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
        if (!this->pRemoteCharacteristic)
        { /*...*/
            this->pClient->disconnect();
            return false;
        }
        Serial.println("Found Characteristic");

        if (this->pRemoteCharacteristic->canNotify())
        { // Use member pRemoteCharacteristic
            auto notifyCallbackWrapper =
                [this](NimBLERemoteCharacteristic *pChr, uint8_t *pData, size_t length, bool isNotify)
            {
                if (this->clientCallBack)
                {
                    this->clientCallBack(pChr, pData, length, isNotify);
                }
                else
                {
                    Serial.println("Notification received, but no client callback registered!");
                }
            };

            if (this->pRemoteCharacteristic->subscribe(true, notifyCallbackWrapper))
            { // Use member
                Serial.println("Subscribed to notifications");
                connectedFlg = true;
                return true;
            }
            else
            { /*...*/
                this->pClient->disconnect();
                return false;
            }
        }
        else
        { /*...*/
            this->pClient->disconnect();
            return false;
        }
    }
    else
    {
        Serial.println("Failed to connect to server");
        this->pRemoteCharacteristic = nullptr; // Reset member pointer
        return false;
    }
}

// Connect using MAC String
bool MyBle::connectToMac(String macAddress)
{
    // Use std::string and address type
    NimBLEAddress address(std::string(macAddress.c_str()), BLE_ADDR_PUBLIC);
    return connectToServer(address);
}

// Send String (Client or Server)
void MyBle::sendString(String str)
{
    if (isClientMode)
    {
        // Use member pRemoteCharacteristic
        if (this->pRemoteCharacteristic && this->pRemoteCharacteristic->canWrite())
        {
            if (!this->pRemoteCharacteristic->writeValue(str, false))
            { // false = no response
                Serial.println("Client write failed!");
            }
        }
        else
        {
            Serial.println("Warning: Client not connected or characteristic cannot be written.");
        }
        return;
    }
    // Server mode: Use the queue
    if (xSemaphoreTake(sendQueueMutex, portMAX_DELAY) == pdTRUE)
    {
        sendQueue.push(str);
        xSemaphoreGive(sendQueueMutex);
    }
    else
    {
        Serial.println("!!! Error: Could not take send queue mutex in sendString !!!");
    }
}

// Send C-String Data
void MyBle::sendData(const char *data) { sendString(String(data)); }

// Send Long String (Client or Server)
///*
// Method in myNimBle.cpp
void MyBle::sendLongString(String str)
{
    int maxPayload = 20; // Default payload size

    // Determine max payload based on connection MTU
    if (isClientMode && this->pClient && this->pClient->isConnected())
    {
        maxPayload = this->pClient->getMTU() - 5; // Client overhead
        if (maxPayload <= 0)
            maxPayload = 20; // Ensure valid minimum
        Serial.printf("[sendLongString] Client Mode: MTU=%d, Calculated Max Payload=%d\n", this->pClient->getMTU(), maxPayload);
    }
    else if (!isClientMode && this->pServer)
    {
        int currentMtu = NimBLEDevice::getMTU();
        maxPayload = currentMtu > 0 ? (currentMtu - 3) : 20; // Server overhead (notify)
        if (maxPayload <= 0)
            maxPayload = 20; // Ensure valid minimum
        Serial.printf("[sendLongString] Server Mode: MTU=%d, Calculated Max Payload=%d\n", currentMtu, maxPayload);
    }
    else
    {
        Serial.printf("[sendLongString] Warning: Could not determine MTU, using default payload: %d\n", maxPayload);
    }

    Serial.printf("[sendLongString] Sending long string (%s, %d bytes total), chunk size ~%d\n",
                  isClientMode ? "Client" : "Server", str.length(), maxPayload);

    int from = 0;
    int totalChunks = (str.length() + maxPayload - 1) / maxPayload; // Calculate total chunks
    int chunkNum = 1;

    while (from < str.length())
    {
        int len = std::min(maxPayload, (int)str.length() - from);
        String strChunk = str.substring(from, from + len);

        Serial.printf("[sendLongString] Sending chunk %d/%d (%d bytes): %s\n",
                      chunkNum, totalChunks, len, strChunk.c_str());

        sendString(strChunk); // Call the regular sendString function
        Serial.printf("[sendLongString] Chunk %d sent. Waiting %dms...\n", chunkNum, 20);

        from += len;
        chunkNum++;

        // Apply delay unconditionally after sending each chunk
        vTaskDelay(pdMS_TO_TICKS(20)); // Adjust this delay (e.g., 20-50ms) as needed
    }
    Serial.println("[sendLongString] Finished sending long string.");
}
//*/
/*
void MyBle::sendLongString(String str)
{
    // #define HeaderSize 4 // bytes
    //   const int CHUNKSIZE = NimBLEDevice::getMTU() - HeaderSize; // Define the chunk size
    const int CHUNKSIZE = 250; // Define the chunk size
    while (sendQueue.size() > 0)
    {
        vTaskDelay(pdTICKS_TO_MS(2));
    }
    while (str.length() > 0)
    {
        String strChunk = str.substring(0, CHUNKSIZE);
        str = str.substring(CHUNKSIZE);
        Serial.println(strChunk);
        sendString(strChunk);
        // justSend(strChunk);
        // vTaskDelay(pdMS_TO_TICKS(100));
        // Serial.println("A:"+strChunk);
        // Saman : i checked for this and it actually do the process til the end
    }
}
*/

// Disconnect (Client or Server)
void MyBle::disconnect()
{
    if (isClientMode && this->pClient && this->pClient->isConnected())
    { // Use member pClient
        Serial.println("Disconnecting client...");
        this->pClient->disconnect();
        connectedFlg = false;
        this->pRemoteCharacteristic = nullptr; // Reset member pointer
    }
    else if (!isClientMode && this->pServer)
    { // Use member pServer
        int count = this->pServer->getConnectedCount();
        Serial.printf("Disconnecting %d server clients...\n", count);
        if (count > 0)
        {
            // Simplified: Disconnect handle 0. Might not disconnect all.
            // Reliable disconnect-all needs iterating handles/peers if API allows.
            Serial.println("Attempting to disconnect client with handle 0...");
            this->pServer->disconnect(0);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow callbacks time
        connectedFlg = (this->pServer->getConnectedCount() > 0);
    }
    // Ensure flag is false if objects don't exist
    if (!this->pClient && !this->pServer)
        connectedFlg = false;
}

// Check Connection Status
bool MyBle::isConnected()
{
    // Use member pointers
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

// --- Other methods largely unchanged, check member pointer access if needed ---
void MyBle::pause()
{
    runFlg = false;
    Serial.println("MyBle paused");
}
void MyBle::resume()
{
    runFlg = true;
    Serial.println("MyBle resumed");
}
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
    if (isClientMode)
        return false;
    bool isBusy = false;
    if (xSemaphoreTake(sendQueueMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        isBusy = !sendQueue.empty();
        xSemaphoreGive(sendQueueMutex);
    }
    else
    {
        isBusy = true;
    }
    return isBusy;
}

String MyBle::directRead()
{
    if (!directReadingFlg)
        return "";
    TickType_t startTick = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(5000);
    while (!dataReady)
    {
        if (xTaskGetTickCount() - startTick > timeoutTicks)
            return "";
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    dataReady = false;
    return strBLE;
}

// Corrected deleteAllBonds function in myNimBle.cpp

void MyBle::deleteAllBonds()
{
    Serial.println("Deleting all bonds...");

    // Get the number of bonded devices
    int numBonds = NimBLEDevice::getNumBonds();
    Serial.printf("Found %d bonded devices.\n", numBonds);

    if (numBonds > 0)
    {
        NimBLEAddress *pAddrs = new NimBLEAddress[numBonds]; // Create temporary array to store addresses

        // Get all addresses first because deleting might change indices
        for (int i = 0; i < numBonds; i++)
        {
            pAddrs[i] = NimBLEDevice::getBondedAddress(i);
            Serial.println("Found Bond: " + String(pAddrs[i].toString().c_str()));
        }

        Serial.println("Now deleting bonds...");
        // Now delete them using the stored addresses
        for (int i = 0; i < numBonds; i++)
        {
            Serial.println("Deleting bond for: " + String(pAddrs[i].toString().c_str()));
            if (!NimBLEDevice::deleteBond(pAddrs[i]))
            {
                Serial.println("...Failed to delete bond.");
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between deletions
        }

        delete[] pAddrs; // Clean up temporary array

        Serial.println("Bond deletion complete. Recommend restarting ESP32.");
    }
    else
    {
        Serial.println("No bonds to delete.");
    }
}
// Scan Result Callback
void MyBle::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
    Serial.print("Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    if (advertisedDevice->isAdvertisingService(serviceUUID))
    {
        Serial.println("Found target service!");
        bleAddTmp = advertisedDevice->getAddress().toString().c_str(); // Static temp storage

        // Stop scan using isScanning() check
        if (NimBLEDevice::getScan()->isScanning())
        { // Use isScanning()
            NimBLEDevice::getScan()->stop();
            Serial.println("Scan stopped.");
        }
    }
}

// === NEW: setPassKey ===
void MyBle::setPassKey(uint32_t _password, bool wipe)
{
    passKey = _password;
    NimBLEDevice::setSecurityPasskey(_password);
    if (wipe)
    {
        deleteAllBonds();
        disconnectAllWhiteList();
        clearwhitelist();
        clearwaitelist();
    }
}

// === NEW: getPassKey ===
uint32_t MyBle::getPassKey() const
{
    return passKey;
}

// list managements
void MyBle::addToWhiteList(const NimBLEAddress &address, const NimBLEConnInfo &connInfo)
{
    for (auto &client : whiteList)
    {
        if (client.MacAdd.equals(address))
        {
            client.connInfo = connInfo;
            return;
        }
    }

    if (whiteList.size() < WHITE_LIST_SIZE)
    {
        whiteList.push_back({address, connInfo});
    }
    else
    {
        Serial.println("Whitelist is full!");
    }
}
void MyBle::addToWaiteList(const NimBLEAddress &address, const NimBLEConnInfo &connInfo)
{
    for (auto &client : waiteList)
    {
        if (client.MacAdd.equals(address))
        {
            client.connInfo = connInfo;
            return;
        }
    }

    if (waiteList.size() < WAITE_LIST_SIZE)
    {
        waiteList.push_back({address, connInfo, millis(), 0, 0, false});
    }
    else
    {
        Serial.println("Waitelist is full!");
    }
}
bool MyBle::isInWhiteList(const NimBLEAddress &address)
{
    for (const auto &client : whiteList)
    {
        if (client.MacAdd.equals(address))
        {
            return true;
        }
    }
    return false;
}
bool MyBle::isInWaiteList(const NimBLEAddress &address)
{
    for (const auto &client : waiteList)
    {
        if (client.MacAdd.equals(address))
        {
            return true;
        }
    }
    return false;
}
void MyBle::removeFromWaiteList(const NimBLEAddress &address)
{
    for (auto it = waiteList.begin(); it != waiteList.end(); ++it)
    {
        if (it->MacAdd.equals(address))
        {
            waiteList.erase(it);
            return;
        }
    }
}
void MyBle::removeFromWhiteList(const NimBLEAddress &address)
{
    for (auto it = whiteList.begin(); it != whiteList.end(); ++it)
    {
        if (it->MacAdd.equals(address))
        {
            whiteList.erase(it);
            return;
        }
    }
}
int MyBle::getWhiteLisindex(const NimBLEAddress &address)
{
    for (int i = 0; i < whiteList.size(); i++)
    {
        if (whiteList[i].MacAdd.equals(address))
        {
            return i;
        }
    }
    return -1;
}
int MyBle::getWaiteLisindex(const NimBLEAddress &address)
{
    for (int i = 0; i < waiteList.size(); i++)
    {
        if (waiteList[i].MacAdd.equals(address))
        {
            return i;
        }
    }
    return -1;
}
void MyBle::setResponsFromClient(const NimBLEAddress &address, bool userIsEnteringPass)
{
    int index = getWaiteLisindex(address);
    if (index != -1)
    {
        waiteList[index].clientResponsing = true;
    }
}
bool MyBle::getResponsFromClient(const NimBLEAddress &address)
{
    int index = getWaiteLisindex(address);
    if (index != -1)
    {
        return waiteList[index].clientResponsing;
    }
    return false;
}

void MyBle::authenticate(NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)
{
    static String accumulatedData;
    // Convert received data to String
    String receivedData(reinterpret_cast<char *>(pData), length);
    accumulatedData += receivedData;
    int endPos;
    while ((endPos = accumulatedData.indexOf('\n')) != -1)
    {
        String command = accumulatedData.substring(0, endPos); // Extract command
        accumulatedData.remove(0, endPos + 1);                 // Remove the processed command
        // Command processing
        if (command.startsWith("PASSKEY="))
        {
            uint32_t _passKey = command.substring(8).toInt();
            if (_passKey == passKey)
            {
                // add to whitelist
                addToWhiteList(connInfo.getAddress(), connInfo);
                removeFromWaiteList(connInfo.getAddress());
                //send successfull to the peer
                sendStringToMac("PASSKEY_ACCEPTED\n", connInfo);
                Serial.printf("PASSKEY:%d is correct. %s added to whitelist\n", _passKey, connInfo.getAddress().toString().c_str());
            }
            else
            {
                Serial.printf("PASSKEY:%d is incorrect\n", _passKey);
                // send to client "InvalidPassword\n"
                sendStringToMac("INVALID_PASSWORD\n", connInfo);
                // get index of waiteList
                int index = getWaiteLisindex(connInfo.getAddress());
                if (index != -1)
                {
                    waiteList[index].passwordRetryCount++;
                    if (waiteList[index].passwordRetryCount > MAX_RETRY_ATTEMPTS)
                    {
                        if (pServer)
                        {
                            pServer->disconnect(connInfo.getConnHandle());
                            removeFromWaiteList(connInfo.getAddress());
                            Serial.printf("Too many retries. %s disconnected and removed from waitlist\n", connInfo.getAddress().toString().c_str());
                        }
                    }
                    else
                    {
                        Serial.printf("PASSKEY:%d is incorrect. Retry count: %d\n", _passKey, waiteList[index].passwordRetryCount);
                    }
                }
            }
        }
        else if (command.startsWith("WAITING_FOR_PASSKEY"))
        {
            setResponsFromClient(connInfo.getAddress(), true);
            Serial.printf("WAITING_FOR_PASSKEY: %s\n", connInfo.getAddress().toString().c_str());
        }
        else
        {
            Serial.printf("Unknown command: %s\n", command.c_str());
        }
    }
}
