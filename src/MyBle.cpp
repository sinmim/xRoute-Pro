#include "myBle.h"

static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLECharacteristic *pServerCharacteristic;

bool MyBle::connectedFlg = false;
bool MyBle::newConnectionMade = false;
BLEAddress *MyBle::pServerAddress = nullptr;
String MyBle::bleAddTmp = "";

MyBle::MyBle(bool clientMode) : isClientMode(clientMode)
{
    sendQueueMutex = xSemaphoreCreateMutex();
    xTaskCreate(sendTask, "SendTask", 4096, this, 1, NULL);
}

MyBle::~MyBle()
{
    vSemaphoreDelete(sendQueueMutex);
}

void MyBle::begin(std::function<void(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb)
{
    clientCallBack = cb;
    if (isClientMode)
    {
        BLEDevice::init("LabobinxSmart");
        pClient = BLEDevice::createClient();
    }
}

void MyBle::beginServer(std::function<void(BLECharacteristic *pCharacteristic, uint8_t *pData, size_t length)> cb)
{
    serverCallBack = cb;
    if (!isClientMode)
    {
        BLEDevice::init("LabobinxSmart");
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        BLEService *pService = pServer->createService(serviceUUID);
        pServerCharacteristic = pService->createCharacteristic(
            charUUID,
            BLECharacteristic::PROPERTY_READ |
                BLECharacteristic::PROPERTY_WRITE |
                BLECharacteristic::PROPERTY_NOTIFY |
                BLECharacteristic::PROPERTY_INDICATE);

        pServerCharacteristic->addDescriptor(new BLE2902());
        pService->start();

        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(serviceUUID);
        pAdvertising->setScanResponse(false);
        pAdvertising->setMinPreferred(0x06); // Helps with iPhone connections
        pAdvertising->setMinPreferred(0x12); // Default value
        pAdvertising->start();
        Serial.println("Advertising started...");
    }
}

bool MyBle::connectToServer(BLEAddress pAddress)
{
    if (!isClientMode)
        return false;

    connectedFlg = false;
    disconnect();
    pClient->setClientCallbacks(new MyClientCallback());
    pClient->connect(pAddress);
    vTaskDelay(pdTICKS_TO_MS(1000));
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);

    if (pRemoteService == nullptr)
    {
        return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr)
    {
        return false;
    }

    pRemoteCharacteristic->registerForNotify(clientCallBack);
    connectedFlg = true;
    return true;
}

bool MyBle::connectToMac(String macAddress)
{
    if (!isClientMode)
        return false;

    pServerAddress = new BLEAddress(macAddress.c_str());
    return connectToServer(*pServerAddress);
}

void MyBle::disconnect()
{
    if (isClientMode && pClient != nullptr && pClient->isConnected())
    {
        pClient->disconnect();
        pClient = nullptr;
    }
    connectedFlg = false;
}

bool MyBle::isConnected()
{
    return connectedFlg;
}

void MyBle::sendString(String str)
{
    if (xSemaphoreTake(sendQueueMutex, portMAX_DELAY))
    {
        sendQueue.push(str);
        xSemaphoreGive(sendQueueMutex);
    }
}

void MyBle::sendData(const char *data)
{
    if (isConnected())
    {
        if (isClientMode && pRemoteCharacteristic != nullptr)
        {
            pRemoteCharacteristic->writeValue(data, strlen(data));
        }
        else if (!isClientMode && pServerCharacteristic != nullptr)
        {
            pServerCharacteristic->setValue(data);
            pServerCharacteristic->notify();
        }
    }
    else
    {
        Serial.println("NotConnected");
    }
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

void MyBle::sendTask(void *parameter)
{
    MyBle *myBle = static_cast<MyBle *>(parameter);
    for (;;)
    {
        if (!myBle->sendQueue.empty())
        {
            if (xSemaphoreTake(myBle->sendQueueMutex, portMAX_DELAY))
            {
                String str = myBle->sendQueue.front();
                myBle->sendQueue.pop();
                xSemaphoreGive(myBle->sendQueueMutex);

                myBle->sendData(str.c_str());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
