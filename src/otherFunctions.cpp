#include <Arduino.h>
#include <esp_system.h>
#include <WiFi.h>
#include <SHA256.h>
#include "otherFunctions.h"
#include "relay.h"
#include "BLESerial.h"
#include "BluetoothSerial.h"
#include "main.h"
#include "stdio.h"
#include <SPIFFS.h>
#include <FS.h>

extern BluetoothSerial SerialBT;
extern relConfig RELAYS;
extern float dimTmp[];
extern float dimLimit[];
const char *secret_key = "inGodWeTrust";

bool checkPass(uint64_t uid, const char *pass)
{
  // Combine the chip ID with the secret key
  char seed[32];
  snprintf(seed, sizeof(seed), "%016llX%s", uid, secret_key);

  // Generate a SHA-256 hash from the seed
  SHA256 sha256;
  uint8_t hash[32];
  sha256.update((const uint8_t *)seed, strlen(seed));
  sha256.finalize(hash, sizeof(hash));

  // Convert the hash to a registration code
  char code[17];
  snprintf(code, sizeof(code), "%02X%02X%02X%02X%02X%02X%02X%02X",
           hash[0], hash[1], hash[2], hash[3], hash[4], hash[5], hash[6], hash[7]);

  // Compare the registration code with the provided password
  return strcmp(pass, code) == 0;
}
void SetNextion(unsigned int pos, float *dimTmp, float *dimLimit)
{
  char str[64];

  sprintf(str, "\xFF\xFF\xFF\xFF\xFF\xFF");
  SendToAll(str);

  for (int i = 0; i < 16; i++)
  {
    sprintf(str, "M.sw%d.val=%d\xFF\xFF\xFF", i + 1, RELAYS.relPos & (1UL << RELAYS.cnfgLookup[i]) ? 1 : 0);
    SendToAll(str);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  for (int i = 0; i < 7; i++)
  {
    float val = (dimTmp[i] / (32768 * dimLimit[i])) * 255;
    sprintf(str, "DIMER%d.val=%d\xFF\xFF\xFF", i + 1, (int)val);
    if (SerialBT.connected())
    {
      SerialBT.println(str);
    }
  }

  for (int i = 0; i < 7; i++)
  {
    float val = (dimTmp[i] / (32768 * dimLimit[i])) * 255;
    sprintf(str, "APDIM%d.val=%d\n", i + 1, (int)val);
    SendToAll(str);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void SendToAll(char *str)
{
  BLEsend(str);
  if (SerialBT.connected())
    SerialBT.println(str);
}
void SendToAll(const char *str)
{
  SendToAll((char *)str);
}
void SerialPrint(char *str)
{
  Serial.print(str);
}
double psiToMeters(double x)
{
  double terms[] = {
      2.9555552436857372e+004,
      -1.7883867989464394e+004,
      9.8312047316532298e+003,
      -3.7605201110388034e+003,
      9.5496784858940555e+002,
      -1.6250311741869669e+002,
      1.8559912122100705e+001,
      -1.4009705117575890e+000,
      6.6928561584190832e-002,
      -1.8309852995320351e-003,
      2.1837249289504070e-005};
  size_t csz = sizeof terms / sizeof *terms;
  double t = 1;
  double r = 0;
  for (int i = 0; i < csz; i++)
  {
    r += terms[i] * t;
    t *= x;
  }
  return r;
}
void printTaskResourceUsage(int interval)
{
  int aa;
  if (aa++ % interval == 0)
  {
    Serial.print("TotalHeap (Byte):" + String(xPortGetFreeHeapSize()));                            // Measuring remaining Heap in bytes
    Serial.println(" This Task Stack Hight (Words):" + String(uxTaskGetStackHighWaterMark(NULL))); //
  }
}
void giveMeMacAdress()
{
  // Initialize BLE
  // BLEDevice::init("");
  // Print ESP32 BLE MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  //Serial.print("BLE MAC Address: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", mac[i]);
    if (i < 5)
    {
      Serial.print(":");
    }
  }
  Serial.println();
}
int getDimVal(int n)
{
  float val = (dimTmp[n] / (32768 * dimLimit[n])) * 255;
  return (int)val;
}
bool SaveStringToFile(String str, String path)
{
  Serial.println("Creating ConfigFile from: " + str);
  // Open file for writing
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file)
  {
    return false;
  }
  // Write to the file
  file.write((const uint8_t *)str.c_str(), str.length());
  file.close();
  // Open file for reading
  file = SPIFFS.open(path, FILE_READ);
  if (!file)
  {
    return false;
  }
  // Read from the file
  String fileContent = file.readString();
  Serial.println("Inside Saved File: " + fileContent + "; END");
  file.close();
  return true;
}
String readStringFromFile(String path)
{
  //Serial.println("readStringFromFile:" + path);
  File file = SPIFFS.open(path, FILE_READ);
  if (!file)
  {
    Serial.println("Error Reading: " + path);
    return "";
  }
  else
  {
    // Read from the file
    String fileContent = file.readString();
    file.close();
    return fileContent;
  }
}
