#ifndef MY_BLE_H
#define MY_BLE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <queue>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class MySecurity : public BLESecurityCallbacks
{
  uint32_t onPassKeyRequest()
  {
    Serial.println("onPassKeyRequest");
    return 123456;
  }
  void onPassKeyNotify(uint32_t pass_key) {}
  bool onConfirmPIN(uint32_t pass_key)
  {
    Serial.println("onConfirmPIN");
    return true;
  }
  bool onSecurityRequest()
  {
    Serial.println("onSecurityRequest");
    return true;
  }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl)
  {
    if (auth_cmpl.success)
    {
      Serial.println("Authentication successful");
    }
  }
};

class MyBle
{
private:
  bool isClientMode;
  BLEClient *pClient;
  BLEServer *pServer;
  BLESecurity *pSecurity;
  BLECharacteristic *pCharacteristic;
  static BLEAddress *pServerAddress;
  bool runFlg = true;
  static bool connectedFlg;
  static bool newConnectionMade;
  std::function<void(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> clientCallBack;
  std::function<void(BLECharacteristic *pCharacteristic, uint8_t *pData, size_t length)> serverCallBack;
  std::queue<String> sendQueue;
  SemaphoreHandle_t sendQueueMutex;
  static void sendTask(void *parameter);

public:
  static String bleAddTmp;

  MyBle(bool clientMode = true);
  ~MyBle();
  void begin(std::function<void(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb);
  void beginServer(std::function<void(BLECharacteristic *pCharacteristic, uint8_t *pData, size_t length)> cb);
  bool connectToServer(BLEAddress pAddress);
  bool connectToMac(String macAddress);
  void sendString(String str);
  void sendData(const char *data);
  void disconnect();
  bool isConnected();
  void pause();
  void resume();
  bool isRunning();
  bool isNewConnection()
  {
    if (newConnectionMade)
    {
      newConnectionMade = false;
      return true;
    }
    else
      return false;
  }
  class MyClientCallback : public BLEClientCallbacks
  {
    void onConnect(BLEClient *pclient)
    {
      newConnectionMade = true;
    }
    void onDisconnect(BLEClient *pclient)
    {
      connectedFlg = false;
    }
  };

  class MyServerCallbacks : public BLEServerCallbacks
  {
    void onConnect(BLEServer *pServer)
    {
      connectedFlg = true;
      Serial.println("Client connected");
    }
    void onDisconnect(BLEServer *pServer)
    {
      connectedFlg = false;
      Serial.println("Client disconnected");
      pServer->getAdvertising()->start(); // Restart advertising
      Serial.println("Advertising restarted...");
    }
  };
};

#endif
