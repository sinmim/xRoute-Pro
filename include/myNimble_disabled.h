#ifndef MY_NIM_BLE_H
#define MY_NIM_BLE_H

#define ENABLE_MY_NIM_BLE_DEBUG
#ifdef ENABLE_MY_NIM_BLE_DEBUG
#define NIMBLE_LOG(format, ...) printf("[NimBle] " format "\n", ##__VA_ARGS__)
#else
#define NIMBLE_LOG(format, ...)
#endif

#include "myNimbleConfig.h"
#include <NimBLEDevice.h>
#include <Arduino.h>
#include <queue>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector>
#include <string>

struct whiteListInfo
{
  NimBLEAddress MacAdd;
  NimBLEConnInfo connInfo;
};

struct waiteListInfo
{
  NimBLEAddress MacAdd;
  NimBLEConnInfo connInfo;
  unsigned long startMilis;
  int passwordRetryCount;
  int retyForRespons;
  bool clientResponsing;
  std::string accumulatedData;
};

struct BleWriteRequest
{
  NimBLEAddress address;
  uint16_t connHandle;
  std::string data;
};


class MyBle
{
private:
  static const int WHITE_LIST_SIZE = 10;
  static const int WAITE_LIST_SIZE = 10;
  static const int MAX_RETRY_ATTEMPTS = 3;
  static const int AUT_TIME_OUT = 5000;
  static const int WAITE_FOR_USER_ENTERING_PASSKEY = 20000;

  static std::vector<whiteListInfo> whiteList;
  static std::vector<waiteListInfo> waiteList;
  
  SemaphoreHandle_t listMutex;
  QueueHandle_t bleWriteQueue;

  static void timeOutTask(void *parameter);
  static void sendTask(void *parameter);
  static void bleWriteTask(void *parameter);

  uint32_t passKey = 0;
  bool isClientMode;
  NimBLEClient *pClient = nullptr;
  NimBLEServer *pServer = nullptr;
  static NimBLEAddress *pServerAddress;
  bool runFlg = true;
  static bool connectedFlg;
  static bool newConnectionMade;

  NimBLERemoteCharacteristic *pRemoteCharacteristic = nullptr; 
  NimBLECharacteristic *pServerCharacteristic = nullptr;      

  std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> clientCallBack;
  std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> serverCallBack;

  std::queue<String> sendQueue;
  SemaphoreHandle_t sendQueueMutex;

  String strBLE;
  bool directReadingFlg = false;
  bool dataReady = false;

  class MyClientCallback : public NimBLEClientCallbacks
  {
    MyBle *pMyBleInstance;
  public:
    MyClientCallback(MyBle *parent) : pMyBleInstance(parent) {}
    void onConnect(NimBLEClient *pclient) override;
    void onDisconnect(NimBLEClient *pclient, int reason) override;
  };

  class MyServerCallbacks : public NimBLEServerCallbacks
  {
    MyBle *pMyBleInstance;
  public:
    MyServerCallbacks(MyBle *parent) : pMyBleInstance(parent) {}
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override;
    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override;
    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override;
    // FIXED: Removed 'override' as these methods do not override a base class method
    uint32_t onPassKeyRequest();
    bool onConfirmPIN(uint32_t pass_key);
  };

  class MyCallbacks : public NimBLECharacteristicCallbacks
  {
  private:
    MyBle &parent;
  public:
    MyCallbacks(MyBle &ble) : parent(ble) {}
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override;
  };

protected:
  void onResult(NimBLEAdvertisedDevice *advertisedDevice);
  void authenticate(const NimBLEAddress &address, uint16_t connHandle, const std::string &data);

public:
  static String bleAddTmp;
  String deviceCode;
  String getDeviceCode() const { return deviceCode; }

  MyBle(bool clientMode = true);
  ~MyBle();

  void begin(std::function<void(NimBLERemoteCharacteristic *pNimBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)> cb);
  void beginServer(std::function<void(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint8_t *pData, size_t length)> cb);
  NimBLECharacteristic *getServerCharacteristic() const;
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
  bool isSendQueueBusy();
  void justSend(String data);
  void sendLongString(String str);
  void startDirectRead() { directReadingFlg = true; }
  void stopDirectRead() { directReadingFlg = false; }
  String directRead();
  void deleteAllBonds();
  void setPassKey(uint32_t _password, bool wipe = false);
  uint32_t getPassKey() const;
  
  void addToWhiteList(const NimBLEAddress &address, const NimBLEConnInfo &connInfo);
  void addToWaiteList(const NimBLEAddress &address, const NimBLEConnInfo &connInfo);
  bool isInWhiteList(const NimBLEAddress &address);
  bool isInWaiteList(const NimBLEAddress &address);
  int getWaiteLisindex(const NimBLEAddress &address);
  int getWhiteLisindex(const NimBLEAddress &address);
  void removeFromWaiteList(const NimBLEAddress &address);
  void removeFromWhiteList(const NimBLEAddress &address);
  void clearwhitelist();
  void clearwaitelist();
  void disconnectAllWhiteList();
  
  void sendStringToMac(String str, uint16_t connHandle);

  void setResponsFromClient(const NimBLEAddress &address, bool userIsEnteringPass);
  bool getResponsFromClient(const NimBLEAddress &address);
};

#endif // MY_NIM_BLE_H