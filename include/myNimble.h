#ifndef MY_NIM_BLE_H
#define MY_NIM_BLE_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <queue>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class MyBle
{
private:
  u_int32_t password = 123456;
  bool isClientMode;
  NimBLEClient *pClient;
  NimBLEServer *pServer;
  NimBLECharacteristic *pCharacteristic;
  static NimBLEAddress *pServerAddress;
  bool runFlg = true;
  static bool connectedFlg;
  static bool newConnectionMade;
  std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> clientCallBack;
  std::function<void(NimBLECharacteristic *pCharacteristic, uint8_t *pData, size_t length)> serverCallBack;
  std::queue<String> sendQueue;
  SemaphoreHandle_t sendQueueMutex;
  static void sendTask(void *parameter);
  String strBLE;
  bool directReadingFlg = false;
  bool dataReady = false;

protected:
  void onResult(NimBLEAdvertisedDevice *advertisedDevice); // Ensure this is declared
public:
  static String bleAddTmp;

  MyBle(bool clientMode = true);
  ~MyBle();
  void begin(std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb);
  void beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, uint8_t *pData, size_t length)> cb);
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
  void justSend(String str);
  void setPass(u_int32_t _password);
  bool isSendQueueBusy();
  void sendLongString(String str);
  void startDirectRead()
  {
    directReadingFlg = true;
  }
  void stopDirectRead()
  {
    directReadingFlg = false;
  }
  String directRead()
  {
    while (!dataReady)
    {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    dataReady = false;
    return strBLE;
  }

  // MyClientCallback class
  class MyClientCallback : public NimBLEClientCallbacks
  {
    void onConnect(NimBLEClient *pclient) override
    {
      newConnectionMade = true;
    }

    void onDisconnect(NimBLEClient *pclient) override
    {
      connectedFlg = false;
    }
  };

  // MyServerCallbacks class
  class MyServerCallbacks : public NimBLEServerCallbacks
  {
    void onConnect(NimBLEServer *pServer) override
    {
      connectedFlg = true;
    }

    void onDisconnect(NimBLEServer *pServer) override
    {
      connectedFlg = false;
    }
  };

  // MyCallbacks class
  class MyCallbacks : public NimBLECharacteristicCallbacks
  {
  private:
    MyBle &parent;

  public:
    MyCallbacks(MyBle &ble) : parent(ble) {}

    void onWrite(NimBLECharacteristic *pCharacteristic) override
    {
      std::string value = pCharacteristic->getValue();
      if (parent.directReadingFlg)
      {
        parent.strBLE = value.c_str();
        parent.dataReady = true;
        return;
      }

      if (parent.serverCallBack)
      {
        parent.serverCallBack(pCharacteristic, (uint8_t *)value.data(), value.length());
      }
    }

    void onRead(NimBLECharacteristic *pCharacteristic) override
    {
      Serial.print("Characteristic ");
      Serial.print(pCharacteristic->getUUID().toString().c_str());
      Serial.print(" was read, value: ");
      Serial.println(pCharacteristic->getValue().c_str());
    }
  };
};

#endif
