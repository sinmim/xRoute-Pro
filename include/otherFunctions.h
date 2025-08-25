#ifndef otherFunctions_h
#define otherFunctions_h
#include <WiFiType.h>
#include <Wire.h>
bool checkPass(uint64_t uid, const char *pass);
void sendToAll(char *str);
void sendToAll(const char *str);
void sendToAll(String str);
void SerialPrint(char *str);
double psiToMeters(double x);
double ACID_SOC_OCV(double V);
double AGM_SOC_OCV(double v);
void printTaskResourceUsage(int interval);
void giveMeMacAdress();
bool SaveStringToFile(String str, String path);
String readStringFromFile(String path);
int getDimVal(int n);
bool wrapJson(const char *inputJson, const char *wrapperKey, String &outputJson);
bool unwrapJson(const char *inputJson, const char *wrapperKey, String &outputJson);
struct RGB_VALS
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t brightness;
};

#include "esp_core_dump.h"
class MyCoreDump
{
public:
  MyCoreDump();
  void delete_core_dump();
  esp_err_t load_core_dump();
  bool print_core_dump_summary_to_string(char *str);
  void print_core_dump();

private:
  esp_core_dump_summary_t summary;
};
void printHex(const char *prefix, const uint8_t *data, size_t len);
String getDeviceCode();
int testWifi(String ssid, String pass);

class I2CDevice
{
public:
  I2CDevice(int address, String name)
  {
    _address = address;
    _name = name;
    _connected = false;
  }
  bool isConnected()
  {
    Wire.beginTransmission(_address);
    _connected = (Wire.endTransmission() == 0);
    return _connected;
  }
  bool lastState()
  {
    return _connected;
  }
  String getName()
  {
    return _name;
  }
private:
  int _address;
  String _name;
  bool _connected;
};

#endif