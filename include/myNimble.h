#ifndef MY_NIM_BLE_H
#define MY_NIM_BLE_H
#include "myNimbleConfig.h"
#include <NimBLEDevice.h>
#include <Arduino.h>
#include <queue>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector> // Include for std::vector

class MyBle
{
private:
  bool isClientMode;
  NimBLEClient *pClient = nullptr;
  NimBLEServer *pServer = nullptr;
  static NimBLEAddress *pServerAddress; // Static for scan result? Review if needed.
  bool runFlg = true;
  static bool connectedFlg;
  static bool newConnectionMade;

  // --- Non-static characteristic pointers ---
  NimBLERemoteCharacteristic *pRemoteCharacteristic = nullptr; // For client mode
  NimBLECharacteristic *pServerCharacteristic = nullptr;       // For server mode

  // Callback function types
  std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> clientCallBack;
  std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> serverCallBack;

  std::queue<String> sendQueue;
  SemaphoreHandle_t sendQueueMutex;
  static void sendTask(void *parameter);
  String strBLE;
  bool directReadingFlg = false;
  bool dataReady = false;

  // --- Inner Callback Classes ---

  class MyClientCallback : public NimBLEClientCallbacks
  {
    MyBle *pMyBleInstance;

  public:
    MyClientCallback(MyBle *parent) : pMyBleInstance(parent) {}

    void onConnect(NimBLEClient *pclient) override
    {
      Serial.println("Client Connected");
      newConnectionMade = true; // Use static for simplicity, or pMyBleInstance->newConnectionMade
    }

    void onDisconnect(NimBLEClient *pclient, int reason) override
    {
      Serial.printf("Client Disconnected, reason=%d\n", reason);
      connectedFlg = false; // Use static for simplicity
      // Reset the characteristic pointer via the parent instance
      if (pMyBleInstance)
      {
        pMyBleInstance->pRemoteCharacteristic = nullptr;
      }
    }
  };

  class MyServerCallbacks : public NimBLEServerCallbacks
  {
    MyBle *pMyBleInstance;

  public:
    MyServerCallbacks(MyBle *parent) : pMyBleInstance(parent) {}

    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
    {
      int count = pServer->getConnectedCount();
      Serial.printf("Device connected: %s, Total: %d\n", connInfo.getAddress().toString().c_str(), count);
      connectedFlg = true;
      newConnectionMade = true;
      if (count < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
      {
        NimBLEDevice::startAdvertising();
      }
    }

    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
    {
      vTaskDelay(pdMS_TO_TICKS(50)); // Allow time for count update
      int count = pServer->getConnectedCount();
      Serial.printf("Device disconnected: %s, Reason: %d, Total: %d\n", connInfo.getAddress().toString().c_str(), reason, count);
      connectedFlg = (count > 0);

      // Optional: Restart advertising if needed
      if (count < CONFIG_BT_NIMBLE_MAX_CONNECTIONS)
      {
        NimBLEDevice::startAdvertising();
      }
    }

    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
    {
      Serial.printf("Authentication complete for %s. Encrypted: %s, Authenticated: %s\n",
                    connInfo.getAddress().toString().c_str(),
                    connInfo.isEncrypted() ? "Yes" : "No",
                    connInfo.isAuthenticated() ? "Yes" : "No");

      // Block specific MAC address logic
      // static NimBLEAddress blockedAddr("d0:9c:7a:cc:51:c9", BLE_ADDR_PUBLIC);
      // if (connInfo.getAddress().equals(blockedAddr))
      // {
      //   Serial.printf("Blocked device %s connected, disconnecting...\n", connInfo.getAddress().toString().c_str());
      //   if (pMyBleInstance && pMyBleInstance->pServer)
      //   {
      //     pMyBleInstance->pServer->disconnect(connInfo.getConnHandle());
      //   }
      // }
    }

    uint32_t onPassKeyRequest() /*override*/
    {
      Serial.println("Server Passkey Request");
      return 382501;
    }

    bool onConfirmPIN(uint32_t pass_key) /*override*/
    {
      Serial.printf("Server Confirm PIN: %u\n", pass_key);
      return true;
    }
  };

  class MyCallbacks : public NimBLECharacteristicCallbacks
  {
  private:
    MyBle &parent;

  public:
    MyCallbacks(MyBle &ble) : parent(ble) {}

    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
      NimBLEAttValue value = pCharacteristic->getValue();
      size_t len = value.length();
      uint8_t *data = (uint8_t *)value.data();

      if (parent.directReadingFlg)
      {
        parent.strBLE = value.c_str();
        parent.dataReady = true;
        return;
      }
      if (parent.serverCallBack)
      {
        parent.serverCallBack(pCharacteristic, connInfo, data, len);
      }
    }

    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
      NimBLEAttValue value = pCharacteristic->getValue();
      Serial.printf("Read request from %s for %s, value: %s\n",
                    connInfo.getAddress().toString().c_str(),
                    pCharacteristic->getUUID().toString().c_str(),
                    value.c_str());
    }

    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
    {
      const char *subType = (subValue == 1) ? "Notifications" : (subValue == 2) ? "Indications"
                                                                                : "None";
      Serial.printf("%s: Client %s %s for UUID: %s\n",
                    connInfo.getAddress().toString().c_str(),
                    (subValue > 0) ? "subscribed" : "unsubscribed",
                    subType,
                    pCharacteristic->getUUID().toString().c_str());
    }
  };

protected:
  // Scan result callback
  void onResult(NimBLEAdvertisedDevice *advertisedDevice);

public:
  void justSend(String data);

  static String bleAddTmp; // Static for scan result? Review if needed.

  MyBle(bool clientMode = true);
  ~MyBle();

  // Public methods
  void begin(std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb);
  void beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> cb); // User CB needs updated signature
  // Getter for the server characteristic (if in server mode)
  NimBLECharacteristic *getServerCharacteristic() const
  {
    if (!isClientMode)
    {
      return pServerCharacteristic; // Return the member pointer
    }
    return nullptr; // Return null if in client mode or not initialized
  }
  bool connectToServer(NimBLEAddress pAddress);
  bool connectToMac(String macAddress);
  void sendString(String str);
  void sendData(const char *data);
  void disconnect();
  bool isConnected();
  void pause();
  void resume();
  bool isRunning();
  bool isNewConnection();
  void setPass(u_int32_t _password);
  bool isSendQueueBusy();
  void sendLongString(String str);

  void startDirectRead() { directReadingFlg = true; }
  void stopDirectRead() { directReadingFlg = false; }
  String directRead();

  void deleteAllBonds();
};

#endif // MY_NIM_BLE_H