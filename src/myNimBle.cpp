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
        // changing buffer size
        NimBLEDevice::setMTU(512);
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
        justSend(strChunk);
        // justSend(strChunk);
        // vTaskDelay(pdMS_TO_TICKS(100));
        // Serial.println("A:"+strChunk);
        // Saman : i checked for this and it actually do the process til the end
    }
}

// void MyBle::sendLongString(String str)
// {
// #define HeaderSize 4 // bytes

//     const int CHUNKSIZE = NimBLEDevice::getMTU() - HeaderSize; // Define the chunk size

//     while (str.length() > 0)
//     {
//         // Take a chunk from the string
//         String strChunk = str.substring(0, CHUNKSIZE);

//         // Remove the chunk from the original string
//         str = str.substring(CHUNKSIZE);

//         // Wait until the queue is not busy
//         while (isSendQueueBusy())
//         {
//             vTaskDelay(pdMS_TO_TICKS(2));
//         }

//         // Send the chunk
//         sendString(strChunk);
//     }
// }

/**
 * @brief In NimBLE for ESP32, the default Maximum Transmission Unit (MTU) size
 *        for the BLE buffer is 23 bytes. This means the payload size, or the
 *        amount of data that can be sent in a single packet, is 20 bytes
 *        because the other 3 bytes are used for the BLE protocol overhead.
 *
 *        However, the MTU can be negotiated and increased up to 255 bytes,
 *        allowing for larger payloads to be sent in a single packet. Keep in
 *        mind that increasing the MTU size doesn't automatically increase the
 *        data rate, as the BLE standard limits the data rate, but it allows
 *        sending more data per packet.
 *
 *        To change the MTU size in NimBLE, you can use:
 *
 *        @code
 *        NimBLEDevice::setMTU(your_desired_mtu_size);
 *        @endcode
 *
 *        For example, to set the MTU to 255 bytes:
 *
 *        @code
 *        NimBLEDevice::setMTU(255);
 *        @endcode
 *
 *        This will allow for a maximum payload of 251 bytes per packet after
 *        accounting for the 4-byte BLE header.
 */

// void MyBle::sendLongString(String str)
// {
//     while (sendQueue.size() > 0)
//     {
//         vTaskDelay(pdMS_TO_TICKS(5));
//     }

//     const int CHUNKSIZE = 256-4; // Define the chunk size
//     while (str.length() > 0)
//     {
//         String strChunk = str.substring(0, CHUNKSIZE);
//         str = str.substring(CHUNKSIZE);
//         justSend(strChunk);

//         vTaskDelay(pdMS_TO_TICKS(200));
//     }
// }

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
