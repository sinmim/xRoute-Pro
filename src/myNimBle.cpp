#include "MyNimBLE.h"

// Constructor
MyNimBLE::MyNimBLE() : pServer(nullptr), pService(nullptr), pCharacteristic(nullptr), dataCallback(nullptr) {}

// Initialize BLE and start advertising
void MyNimBLE::begin(const char *deviceName, const char *passKey, const char *serviceUUID, const char *characteristicUUID, DataReceivedCallback callback)
{
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setSecurityPasskey(atoi(passKey));
    NimBLEDevice::setSecurityAuth(true, true, true);

    pServer = NimBLEDevice::createServer();
    pService = pServer->createService(serviceUUID);

    pCharacteristic = pService->createCharacteristic(
        characteristicUUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    dataCallback = callback;
    pCharacteristic->setCallbacks(new NimBLECharacteristicCallbacks());

    pService->start();
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->start();
}