#ifndef MY_NIM_BLE_H
#define MY_NIM_BLE_H
// for nimble config
// https://h2zero.github.io/esp-nimble-cpp/nimconfig_8h.html

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
  void deletAllBoundedDevices()
  {
    Serial.println("Number of Bounded devices :" + String(NimBLEDevice::getNumBonds()));
    NimBLEDevice::deleteAllBonds();
    Serial.println("Number of Bounded devices :" + String(NimBLEDevice::getNumBonds()));
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
  uint8_t *buffer;
  bool readBytesFlag = false;
  bool readBytes(uint8_t *_buffer, int size, int timeout)
  {
    buffer = _buffer;
    readBytesFlag = true;
    for (int i = 0; i < size; i++)
    {
      buffer[i] = 0;
    }
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
    int connectionCount = 0;
    void onConnect(NimBLEServer *pServer) override
    {
      connectionCount++;
      if (connectionCount > 0)
      {
        connectedFlg = true;
      }
      Serial.println("onConnect :" + String(connectionCount));
      if (connectionCount < 4)
      {
        BLEDevice::startAdvertising();
      }
      else
      {
        Serial.println("Advertising Stopped!");
      }
      //-------managing devices
      // //---
      // int numBonds = NimBLEDevice::getNumBonds();
      // Serial.printf("============LIST============");
      // Serial.printf("Number of bonded devices: %d\n", numBonds);
      // for (int i = 0; i < numBonds; i++)
      // {
      //   NimBLEAddress bondedDevice = NimBLEDevice::getBondedAddress(i);
      //   Serial.printf("Bonded Device %d: %s\n", i + 1, bondedDevice.toString().c_str());
      // }
      // Serial.println("============LIST============");
      // //---
      int numBonds = NimBLEDevice::getNumBonds();
      int maxBonds = 5; // Maximum number of bonded devices allowed + 1 : 5=>4 devices
      if (numBonds >= maxBonds)
      {
        NimBLEAddress oldestBond = NimBLEDevice::getBondedAddress(0);
        NimBLEDevice::deleteBond(oldestBond);
        Serial.printf("Deleted bond for the oldest device: %s\n", oldestBond.toString().c_str());
      }
      // //---
      // numBonds = NimBLEDevice::getNumBonds();
      // Serial.println("============LIST============");
      // Serial.printf("Number of bonded devices: %d\n", numBonds);
      // for (int i = 0; i < numBonds; i++)
      // {
      //   NimBLEAddress bondedDevice = NimBLEDevice::getBondedAddress(i);
      //   Serial.printf("Bonded Device %d: %s\n", i + 1, bondedDevice.toString().c_str());
      // }
      // Serial.println("============LIST============");
    }

    void onDisconnect(NimBLEServer *pServer) override
    {
      connectionCount--;
      if (connectionCount == 0)
      {
        connectedFlg = false;
      }
      Serial.println("onDisconnect :" + String(connectionCount));
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
      // if (parent.readBytesFlag)
      // {
      //   for (int i = 0; i < value.length(); i++)
      //   {
      //     Serial.print(value[i], HEX);
      //     parent.buffer[i] = value[i];
      //   }
      // }

      // Serial.print("RAW:");
      // for (int i = 0; i < pCharacteristic->getDataLength(); i++)
      // {
      //   parent.buffer[i] = *(pCharacteristic->getValue().data() + i);
      //   Serial.print(parent.buffer[i], HEX);
      // }
      // Serial.println("-------------");

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
