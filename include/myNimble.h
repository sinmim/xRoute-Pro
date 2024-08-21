#ifndef MYNIMBLE_H
#define MYNIMBLE_H

#include <NimBLEDevice.h>

// Define the service and characteristic UUIDs here
#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_RXTX "0000ffe1-0000-1000-8000-00805f9b34fb"

// Callback type definition
typedef void (*DataReceivedCallback)(NimBLECharacteristic* pCharacteristic);

class MyNimBLE {
public:
    MyNimBLE();
    void begin(const char* deviceName, const char* passKey, DataReceivedCallback callback);

private:
    NimBLEServer* pServer;
    NimBLEService* pService;
    NimBLECharacteristic* pCharacteristic;
    DataReceivedCallback dataCallback;

    static void onCharacteristicWritten(NimBLECharacteristic* pCharacteristic);
};

#endif // MYNIMBLE_H
