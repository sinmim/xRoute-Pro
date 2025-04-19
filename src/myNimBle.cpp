#include "myNimBle.h"
#include <string> // Include for std::string

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
            NimBLEDevice::setSecurityPasskey(382501);
            NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
            NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

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
            this->pServerCharacteristic = pService->createCharacteristic(
                charUUID,
                NIMBLE_PROPERTY::READ |
                    NIMBLE_PROPERTY::WRITE |
                    NIMBLE_PROPERTY::NOTIFY |
                    NIMBLE_PROPERTY::READ_ENC |
                    NIMBLE_PROPERTY::WRITE_ENC);

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

                // Safe advertising intervals
                // pAdvertising->setMinInterval(0x20); // 32 * 0.625ms = 20ms
                // pAdvertising->setMaxInterval(0x40); // 64 * 0.625ms = 40ms
                // pAdvertising->setMinInterval(160); // 100 ms
                // pAdvertising->setMaxInterval(320); // 200 ms
                // NimBLEAdvertisementData advData;
                // advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
                // advData.setFlags((0x01 << 1)|(0x01 << 2));
                // advData.addServiceUUID(serviceUUID);
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

// Send Task
void MyBle::sendTask(void *param)
{
    MyBle *pMyBle = static_cast<MyBle *>(param);
    if (!pMyBle)
        return;

    while (true)
    {
        String dataToSend = "";
        bool hasData = false;

        // Use member pointer pServerCharacteristic
        if (!pMyBle->isClientMode && pMyBle->pServerCharacteristic != nullptr)
        {
            if (xSemaphoreTake(pMyBle->sendQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                if (!pMyBle->sendQueue.empty())
                {
                    dataToSend = pMyBle->sendQueue.front();
                    pMyBle->sendQueue.pop();
                    hasData = true;
                }
                xSemaphoreGive(pMyBle->sendQueueMutex);
            }
        }
        else if (pMyBle->isClientMode)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Check connection and use member pointer
        if (hasData && pMyBle->isConnected() && pMyBle->pServerCharacteristic != nullptr)
        {
            pMyBle->pServerCharacteristic->setValue(dataToSend);
            pMyBle->pServerCharacteristic->notify();
            vTaskDelay(pdMS_TO_TICKS(50)); // Flow control
        }
        else if (hasData && !pMyBle->isConnected())
        {
            // Serial.println("WARN: Data in queue but not connected: " + dataToSend);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
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
/*
void MyBle::sendLongString(String str)
{
    int maxPayload = 20;
    // Use member pointers for checks
    if (isClientMode && this->pClient && this->pClient->isConnected()) {
         maxPayload = this->pClient->getMTU() - 5;
         if (maxPayload <= 0) maxPayload = 20;
    } else if (!isClientMode && this->pServer) {
         maxPayload = NimBLEDevice::getMTU() > 0 ? (NimBLEDevice::getMTU() - 3) : 20;
         if (maxPayload <= 0) maxPayload = 20;
    }

    Serial.printf("Sending long string (%s, %d bytes), chunk size ~%d\n", isClientMode?"Client":"Server", str.length(), maxPayload);

    int from = 0;
    while (from < str.length()) {
        int len = std::min(maxPayload, (int)str.length() - from);
        String strChunk = str.substring(from, from + len);
        sendString(strChunk); // Handles both modes
        from += len;
        if (isClientMode) vTaskDelay(pdMS_TO_TICKS(20)); // Delay between client chunks
    }
    Serial.println("Finished sending long string.");
}
*/

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
void MyBle::setPass(u_int32_t _password) { NimBLEDevice::setSecurityPasskey(_password); }

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