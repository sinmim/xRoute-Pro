#include <Arduino.h>
#include <esp_system.h>
#include <WiFi.h>
#include <SHA256.h>
#include "otherFunctions.h"
#include "relay.h"
#include "esp_core_dump.h"
// BLE//#include "BLESerial.h"
#include "BluetoothSerial.h"
#include "main.h"
#include "stdio.h"
#include <SPIFFS.h>
#include <FS.h>

// extern BluetoothSerial SerialBT;
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
#include "myNimBle.h"
extern MyBle myBle;

#include "XrouteWsServer.h"
extern XrouteWsServer ws;

void sendToAll(char *str)
{
  // if ble have client send string
  if (myBle.isConnected())
  {
    myBle.sendString(str);
  }
  // if ws has client
  ws.sendToAll(str);
}
void sendToAll(const char *str)
{
  sendToAll((char *)str);
}
void sendToAll(String str)
{
  sendToAll((char *)str.c_str());
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
  Serial.print("BLE MAC Address: ");
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
  // Serial.println("readStringFromFile:" + path);
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
// ——— CONFIGURABLE CAPACITIES ———
// Adjust these if your JSON is larger or smaller than shown below:
static const size_t RAW_CAPACITY = 4096;     // capacity for a “flat” object
static const size_t WRAPPED_CAPACITY = 4096; // capacity for { key: [ <flat> ] }

// ——— wrapJson(…) ———
// Parses `inputJson` (a flat object) and produces `outputJson` such that:
//   outputJson == { "<wrapperKey>": [ <entire inputJson-object> ] }
//
//   wrapperKey: the array-name you want at the top level (e.g. "UiConfig",
//               "MySettings", "Foo", etc.)
//
// Returns true on success (and fills outputJson); false on error.
bool wrapJson(const char *inputJson, const char *wrapperKey, String &outputJson)
{
  // 1) Parse the flat input into a temporary JsonDocument
  StaticJsonDocument<RAW_CAPACITY> sourceDoc;
  DeserializationError err = deserializeJson(sourceDoc, inputJson);
  if (err)
  {
    Serial.printf("wrapJson: deserializeJson() failed: %s\n", err.f_str());
    return false;
  }

  // 2) Create a bigger JsonDocument to hold { wrapperKey: [ { … } ] }
  StaticJsonDocument<WRAPPED_CAPACITY> wrappedDoc;

  // 3) Create a JsonArray under wrapperKey
  //    Equivalent to:  wrappedDoc[wrapperKey] = JsonArray();
  JsonArray arr = wrappedDoc.createNestedArray(wrapperKey);

  // 4) Copy the entire source object into that array
  arr.add(sourceDoc.as<JsonObject>());

  // 5) Serialize out to String
  outputJson = "";
  serializeJson(wrappedDoc, outputJson);
  return true;
}

// ——— unwrapJson(…) ———
// Parses `inputJson` (which must be { "<wrapperKey>": [ { … } ] })
// and extracts the first object inside that array. The result is
//   outputJson == the inner object (flat).
//
// Returns true on success; false if parsing fails, if the key is missing,
// or if the array is empty.
bool unwrapJson(const char *inputJson, const char *wrapperKey, String &outputJson)
{
  // 1) Parse the wrapped JSON
  StaticJsonDocument<WRAPPED_CAPACITY> wrappedDoc;
  DeserializationError err = deserializeJson(wrappedDoc, inputJson);
  if (err)
  {
    Serial.printf("unwrapJson: deserializeJson() failed: %s\n", err.f_str());
    return false;
  }

  // 2) Verify the wrapperKey exists and is an array
  if (!wrappedDoc.containsKey(wrapperKey) || !wrappedDoc[wrapperKey].is<JsonArray>())
  {
    Serial.printf("unwrapJson: \"%s\" array missing or not an array\n", wrapperKey);
    return false;
  }
  JsonArray uiArray = wrappedDoc[wrapperKey];

  // 3) Ensure the array is not empty
  if (uiArray.isNull() || uiArray.size() == 0)
  {
    Serial.printf("unwrapJson: \"%s\" array is empty\n", wrapperKey);
    return false;
  }

  // 4) Grab the first element (a JsonObject)
  JsonObject innerObj = uiArray[0];

  // 5) Copy it into its own (smaller) document, then serialize
  StaticJsonDocument<RAW_CAPACITY> outDoc;
  outDoc.set(innerObj);

  outputJson = "";
  serializeJson(outDoc, outputJson);
  return true;
}

// ___ Class coreDump___
MyCoreDump::MyCoreDump()
{
  esp_core_dump_init();
}
void MyCoreDump::delete_core_dump()
{
  esp_core_dump_image_erase();
}
esp_err_t MyCoreDump::load_core_dump()
{
  return esp_core_dump_get_summary(&summary);
}
bool MyCoreDump::print_core_dump_summary_to_string(char *str)
{
  esp_err_t err = load_core_dump();
  if (err != ESP_OK)
  {
    //str = "Failed to get core dump summary! Error code: 0x%X\n";
    return false;
  }
  else
  {
    // neccesary information that is inough for decoding later on
    sprintf(str, "Task Name: %s\nTask TCB Address: 0x%08X\nException PC: 0x%08X\nDepth: %u\nCorrupted: %s\n",
            summary.exc_task, summary.exc_tcb, summary.exc_pc, summary.exc_bt_info.depth, summary.exc_bt_info.corrupted ? "YES" : "NO");
    return true;
  }
}
void MyCoreDump::print_core_dump()
{
  esp_err_t err = load_core_dump();
  if (err != ESP_OK)
  {
    Serial.printf("Failed to get core dump summary! Error code: 0x%X\n", err);
    return;
  }
  Serial.println("\n========== Core Dump Summary ==========");
  Serial.printf("🧵 Task Name         : %s\n", summary.exc_task);
  Serial.printf("🔗 Task TCB Address  : 0x%08X\n", summary.exc_tcb);
  Serial.printf("💥 Exception PC      : 0x%08X\n", summary.exc_pc);
  Serial.println("\n--- 🔙 Backtrace ---");
  Serial.printf("Depth                : %u frames\n", summary.exc_bt_info.depth);
  Serial.printf("Corrupted            : %s\n", summary.exc_bt_info.corrupted ? "YES" : "NO");
  for (uint32_t i = 0; i < summary.exc_bt_info.depth; i++)
  {
    Serial.printf("  #%02d: 0x%08X\n", i, summary.exc_bt_info.bt[i]);
  }
  Serial.println("\n--- 🧬 App ELF SHA256 ---");
  Serial.print("Hash                 : ");
  for (int i = 0; i < APP_ELF_SHA256_SZ; i++)
  {
    Serial.printf("%02X", summary.app_elf_sha256[i]);
  }
  Serial.println();
  Serial.println("\n--- 🧠 Extra Exception Info ---");
  Serial.printf("Exception Cause      : 0x%08X\n", summary.ex_info.exc_cause);
  Serial.printf("Virtual Address      : 0x%08X\n", summary.ex_info.exc_vaddr);
  Serial.println("\nRegisters (A0–A15):");
  for (int i = 0; i < 16; i++)
  {
    Serial.printf("  A%-2d: 0x%08X\n", i, summary.ex_info.exc_a[i]);
  }
  Serial.println("\nEPCx Registers:");
  for (int i = 0; i < EPCx_REGISTER_COUNT; i++)
  {
    if (summary.ex_info.epcx_reg_bits & (1 << i))
    {
      Serial.printf("  EPC%-2d: 0x%08X\n", i + 1, summary.ex_info.epcx[i]);
    }
  }
  Serial.println("\n--- ⚙️  System Info ---");
  Serial.printf("Core Dump Version    : %u\n", summary.core_dump_version);
  Serial.println("=========================================\n");
}
