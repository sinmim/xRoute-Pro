#ifndef MYNIMBLE_H
#define MYNIMBLE_H

#include <NimBLEDevice.h>
#include <string>

class MyNimBLE : public NimBLECharacteristicCallbacks {
public:
    // Constructor
    MyNimBLE();

    // Initialize BLE and start advertising
    void begin(const char* deviceName, const char* passKey, const char* serviceUUID, const char* characteristicUUID);

    // Handle characteristic writes
    void onWrite(NimBLECharacteristic* pCharacteristic) override;

private:
    // Process received data
    void processReceivedData(const std::string& data);

    NimBLEServer* pServer;
    NimBLEService* pService;
    NimBLECharacteristic* pCharacteristic;
};

#endif // MYNIMBLE_H
