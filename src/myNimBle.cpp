#include "myNimBLE.h"

// Constructor
MyNimBLE::MyNimBLE() : pServer(nullptr), pService(nullptr), pCharacteristic(nullptr)
{
    // Initialize other members if needed
}

// Initialize BLE and start advertising
void MyNimBLE::begin(const char *deviceName, const char *passKey, const char *serviceUUID, const char *characteristicUUID)
{
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setSecurityPasskey(atoi(passKey));
    NimBLEDevice::setSecurityAuth(true, true, true);

    pServer = NimBLEDevice::createServer();
    pService = pServer->createService(serviceUUID);

    pCharacteristic = pService->createCharacteristic(
        characteristicUUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    pCharacteristic->setCallbacks(this); // Set the current instance as the callback handler

    pService->start();
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->start();
}

// Handle characteristic writes
void MyNimBLE::onWrite(NimBLECharacteristic *pCharacteristic)
{
    std::string value = pCharacteristic->getValue(); // Read the value from the characteristic

    // Process the received data here
    processReceivedData(value);
}

// Process received data (defined in the class)
void MyNimBLE::processReceivedData(const std::string &data)
{
    // Handle the data received from the characteristic
    // For example, you could parse the data or trigger specific actions
    Serial.println("Data received: " + String(data.c_str()));
}
