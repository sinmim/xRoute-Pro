#include "myNimBle.h"

static NimBLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static NimBLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");
static NimBLERemoteCharacteristic *pRemoteCharacteristic;
static NimBLECharacteristic *pServerCharacteristic;

bool MyBle::connectedFlg = false;
bool MyBle::newConnectionMade = false;
NimBLEAddress *MyBle::pServerAddress = nullptr;
String MyBle::bleAddTmp = "";

MyBle::MyBle(bool clientMode) : isClientMode(clientMode)
{
    sendQueueMutex = xSemaphoreCreateMutex();
    xTaskCreate(sendTask, "SendTask", 1024 * 4, this, 1, NULL);
}

MyBle::~MyBle()
{
    vSemaphoreDelete(sendQueueMutex);
    if (pServerAddress != nullptr)
    {
        delete pServerAddress;
        pServerAddress = nullptr;
    }
}

void MyBle::begin(std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb)
{
    clientCallBack = cb;
    if (isClientMode)
    {
        NimBLEDevice::init("LabobinxSmart");
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback());
    }
}

void MyBle::beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, uint8_t *pData, size_t length)> cb)
{
    serverCallBack = cb;
    if (!isClientMode)
    {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);                              // Read the MAC address
        char uniqueID[7];                                                 // Buffer for the unique ID (6 hex digits + null terminator)
        snprintf(uniqueID, sizeof(uniqueID), "%02X%02X", mac[4], mac[5]); // Use last 3 bytes

        // Combine the base name with the unique ID
        std::string deviceName = "xRoutePro-";
        deviceName += uniqueID;

        NimBLEDevice::init(deviceName);

        NimBLEDevice::setSecurityAuth(true, true, true);
        NimBLEDevice::setSecurityPasskey(123456);
        NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);

        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        NimBLEService *pService = pServer->createService(serviceUUID);
        pServerCharacteristic = pService->createCharacteristic(
            charUUID,
            NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::WRITE |
                NIMBLE_PROPERTY::NOTIFY |
                NIMBLE_PROPERTY::READ_ENC |
                NIMBLE_PROPERTY::WRITE_ENC);

        // Add NimBLE2904 descriptor
        NimBLE2904 *pDescriptor = (NimBLE2904 *)pServerCharacteristic->createDescriptor(
            NimBLEUUID((uint16_t)0x2904),
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        pDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);

        pServerCharacteristic->setCallbacks(new MyCallbacks(*this));

        pService->start();

        NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(serviceUUID);
        pAdvertising->setScanResponse(true);
        pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
        pAdvertising->setMaxPreferred(0x12);
        pAdvertising->start();
        Serial.println("BLE Server is advertising...");
    }
}

void MyBle::sendTask(void *param)
{
    MyBle *pMyBle = static_cast<MyBle *>(param);

    while (true)
    {
        if (pMyBle->isClientMode || pMyBle->sendQueue.empty())
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (xSemaphoreTake(pMyBle->sendQueueMutex, portMAX_DELAY) == pdTRUE)
        {
            String data = pMyBle->sendQueue.front();
            pMyBle->sendQueue.pop();
            xSemaphoreGive(pMyBle->sendQueueMutex);

            pServerCharacteristic->setValue(data);
            pServerCharacteristic->notify();
            // Serial.println("Sent string: " + data);
        }
    }
}

bool MyBle::connectToServer(NimBLEAddress pAddress)
{
    if (pClient->connect(pAddress))
    {
        pRemoteCharacteristic = pClient->getService(serviceUUID)->getCharacteristic(charUUID);
        if (pRemoteCharacteristic)
        {
            pRemoteCharacteristic->registerForNotify(clientCallBack);
            connectedFlg = true;
            return true;
        }
        else
        {
            pClient->disconnect();
            return false;
        }
    }
    return false;
}

bool MyBle::connectToMac(String macAddress)
{
    NimBLEAddress address(macAddress.c_str());
    return connectToServer(address);
}

void MyBle::sendString(String str)
{
    if (xSemaphoreTake(sendQueueMutex, portMAX_DELAY) == pdTRUE)
    {
        sendQueue.push(str);
        xSemaphoreGive(sendQueueMutex);
    }
}

bool MyBle::isSendQueueBusy()
{
    bool isBusy = false;
    if (xSemaphoreTake(sendQueueMutex, portMAX_DELAY) == pdTRUE)
    {
        isBusy = !sendQueue.empty();
        xSemaphoreGive(sendQueueMutex);
    }
    return isBusy;
}

void MyBle::sendLongString(String str)
{
    const int CHUNKSIZE = 256; // Define the chunk size
    while (str.length() > 0)
    {
        // Take a chunk from the string
        String strChunk = str.substring(0, CHUNKSIZE);

        // Remove the chunk from the original string
        str = str.substring(CHUNKSIZE);

        // Wait until the queue is not busy
        while (isSendQueueBusy())
        {
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        // Send the chunk
        sendString(strChunk);
    }
}

void MyBle::justSend(String str)
{
    pServerCharacteristic->setValue(str);
    pServerCharacteristic->notify();
}

void MyBle::sendData(const char *data)
{
    sendString(String(data));
}

void MyBle::disconnect()
{
    if (isClientMode)
    {
        pClient->disconnect();
    }
    else
    {
        pServer->disconnect(0);
    }
    connectedFlg = false;
}

bool MyBle::isConnected()
{
    return connectedFlg;
}

void MyBle::pause()
{
    runFlg = false;
}

void MyBle::resume()
{
    runFlg = true;
}

bool MyBle::isRunning()
{
    return runFlg;
}

bool MyBle::isNewConnection()
{
    if (newConnectionMade)
    {
        newConnectionMade = false;
        return true;
    }
    return false;
}

void MyBle::onResult(NimBLEAdvertisedDevice *advertisedDevice)
{
    Serial.println("Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID))
    {
        bleAddTmp = advertisedDevice->getAddress().toString().c_str();
        NimBLEDevice::getScan()->stop();
    }
}
