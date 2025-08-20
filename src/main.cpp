//====================================================libraries
#include "xRouteDefs.h"
// defiene as XROUTE_PRO
String Version = "0.9.0";
MyController MyBoard(MyController::XROUTE_PRO);
#define __________________________________________INCLUDES
#ifdef __________________________________________INCLUDES
#include <sdkconfig.h>
#include <esp_bt_device.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "defaultValues.h"
#include "save_load.h"
#include "relay.h"
#include "otherFunctions.h"
#include "ledPwm.h"
#include "dsp.h"
#include "adc.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include <GyverBME280.h>
#include <Adafruit_AHTX0.h>
#include <math.h>
#include <battery.h>
#include <esp_system.h>
#include <WiFi.h>
#include <SHA256.h>
#include <Arduino_LPS22HB.h>
#include "SparkFunLIS3DH.h"
#include <SPIFFS.h>
#include <FS.h>
#include <MyWifi.h>
#endif
#define __________________________________________VAR_DEF
#ifdef __________________________________________VAR_DEF
//*******************VERSION CONTROLS
// userInfo
#include "SettingsStore.h"
#include "userInfoKeys.h"
SettingsStore wifi_WebSocket_Settings("/wifi_WebSocket_Settings.json");
SettingsStore other_Settings("/other_Settings.json");
int uiConfigIndex = 0;
//========Update
#include <Update.h>
//_#include "AESLib.h"
long int updateLen;
#define CHUNK_SIZE 512
// #define CHUNK_SIZE 256
uint8_t dataBuff[CHUNK_SIZE];
//_AESLib aes;
// AES decryption key (16 bytes)
uint8_t key[] = {53, 42, 12, 23, 72, 99, 33, 56, 12, 132, 38, 250, 180, 1, 2, 3};
// IV for AES decryption (16 bytes)
uint8_t iv[] = {3, 8, 2, 5, 0, 1, 8, 0, 9, 12, 79, 72, 3, 4, 6, 0};
// Create an AES128 object for decryption
// uint8_t key[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
//  IV for AES decryption (16 bytes)
// uint8_t iv[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
boolean UpdatingFlg = false;
//========Battery
Battery myBattery;
boolean ampSenisConnected = false;
//========Sensor
#define I2C_BUSSY 1
#define I2C_FREE 0
bool i2cStat = I2C_FREE;
Adafruit_MPU6050 mpu;
#define MPU_DISSCONNECT 0
#define MPU_CONNECTED 1
bool MPU_STATE = MPU_DISSCONNECT;
float accXValue = 0, accYValue = 0, accZValue = 0, len = 0, alpha = 0, accSensitivity = 0, degX = 0, degY = 0;
float accXValueOffset = 0;
float accYValueOffset = 0;
boolean GyroOffsetingFlg = false;
sensors_event_t mpuAcc, mpuGyro, mpuTemp;
void initMPU();
//------------------------------------WS2812
#define LEDS_COUNT 1
#define LEDS_PIN 23
#define CHANNEL 0
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);
#define COLOR_RED 0xff0000
#define COLOR_GREEN 0x00ff00
#define COLOR_BLUE 0x0000ff
#define COLOR_ORANG 0xffa000
#define COLOR_OFF 0x000000
#define COLOR_WHITE 0xffffff
int LED_COLOR = COLOR_OFF;
void ws2812Blink(int color, int times, int interval_ms, float intensity);
//------------------------------------e2prom
void loadSavedValue();
EEpromAdd E2ADD;
//------------------------------------HUMIDITY SENSORS
#define HUM_SENSOR_TYPE_NON 0
#define HUM_SENSOR_TYPE_BME280 1
#define HUM_SENSOR_TYPE_AHT10 2
int humSensorType = HUM_SENSOR_TYPE_BME280;
// BME280
GyverBME280 bme;
// AHT10
uint8_t readStatus = 0;
Adafruit_AHTX0 aht;
//------------------------InternalSensors
float psi = 0;
float altitute = 0;
float psiOffset = 0;
//------------------------BME sensor
float digitalTemp = 0;
float digitalHum = 0;
float digitalPress = 0;
float digitalAlt = 0;
//------------------------POWER
float negv = 0, v = 0, a0 = 0, a1 = 0, a2 = 0, w = 0, b = 0, cwPrcnt = 0, dwPrcnt = 0, gwPrcnt = 0, pt100mv = 0, gsmAnten = 0, battHourLeft = 0;
float VcalCo = 0, volt = 0;
boolean lowVoltageFlg = false;
float amp0Offset = 0;
float A0calCo = 1, amp0 = 0;
float amp1Offset = 0;
float A1calCo = 1, amp1 = 0;
float amp2Offset = 0;
float A2calCo = 1, amp2 = 0;
//--------------Negative Voltage
float NegVoltOffsetSave = 0;
float NegVoltOffset = 0;
float Negvolt = 0;
//--------------Battery
float batteryCap = 0;
float battFullVoltage = 0;
float battEmptyVoltage = 0;
float cableResistance = 0;
//--------------Gauges
// clean water G1
float clnWtr = 0;
float clnWtrMax = 0;
float clnWtrMin = 0;
// Dirty Water G2
float drtWtr = 0;
float drtWtrMax = 0;
float drtWtrMin = 0;
// Grey Water G3
float gryWtr = 0;
float gryWtrMax = 0;
float gryWtrMin = 0;
//--------------PT100
float pt100 = 0;
float PT_mvCal = 0;
// END---------------------------------VOLTAGES
//------------------------------------DIMERS
static bool DimValChanged = false;
float dimTmp[7] = {0, 0, 0, 0, 0, 0, 0};
float dimLPF[7] = {0, 0, 0, 0, 0, 0, 0};
float dimLimit[7] = {0, 0, 0, 0, 0, 0, 0};
// END---------------------------------DIMERS
bool overCrntFlg = false;
//------------------------------------Default Values
int DFLT_V_CAL = 120;
int DFLT_A0_CAL = 47;
int DFLT_A_CAL = 50;
int DFLT_A2_CAL = 60;
int DFLT_BATT_CAP = 100;
int DFLT_PT_MV_CAL = 585;
int DFLT_BATT_FULL_VOLT = 135;
int DFLT_BATT_EMPTY_VOLT = 119;
int DFLT_CABLE_RES_MILI_OHM = 0;
float pressurCalOffset = 0.04F; // psi
// END---------------------------------Default Values
extern relConfig RELAYS;
// BluetoothSerial SerialBT;
int blePass;
// END-----------------------DATA's
#endif
// coreDump
MyCoreDump *coreDump;
//===========================CPU
#include "CPU_Usage.h"
CPU_Usage cpu_monitor;
//===========================Licensing
#include <licensing.h>
const String RegFilePath = "/RegFile.txt";
const String UpTimeFilePath = "/UpTime.txt";
RegDev *xrtLcns;
Leasing *xrtLizing;
//===========================Files
// a std vector of uint8_t
//==============================
const String statesFile = "/LastStates.txt";
#include <ButtonConfig.h>
const String ConfigFile = "/ConfigFile.txt";
#include <defaultConditions.h>
extern const char *defaultCondition;
String confAndCondStrBuffer = "";
const String CondFile = "/CondFile.txt";
//===========================Conditions
#include <vector>
#include "conditions.h"
#include "jsonCondition.h"
std::vector<Conditions> cndtions;
ConditionsReader jsonCon;
//====motors
int motorWay = MOTOR_STOP;
//===========================CLASSES
#include "XrouteAsyncWebSocketServer.h"
XrouteAsyncWebSocketServer ws;
// web sucket
uint64_t chipid;
String GeneralLisence;
class lisence
{
public:
  String name;
  String secretKey;
  char realSerial[18];
  bool State;

public:
  lisence(String lisenceName, String key)
  {
    secretKey = key;
    name = lisenceName;
    State = checkActivation();
    generateSerial();
  }
  bool checkActivation()
  {
    // find the key in the string
    int keyIndex = GeneralLisence.indexOf(name);
    if (keyIndex == -1)
    {
      return false;
    }
    // find the end of the key
    int endKeyIndex = GeneralLisence.indexOf(':', keyIndex);
    if (endKeyIndex == -1)
    {
      return false;
    }
    // find the start of the value
    int valueIndex = endKeyIndex + 1;
    // find the end of the value
    int endValueIndex = GeneralLisence.indexOf(',', valueIndex);
    if (endValueIndex == -1)
    {
      endValueIndex = GeneralLisence.length();
    }
    // extract the value substring
    String valueStr = GeneralLisence.substring(valueIndex, endValueIndex);
    // convert the value string to a boolean
    return valueStr == "True "; // chonke false ye harf bishtare be true space zadim too save
  }
  // Edits a boolean license feature in a string based on a new state.
  void editGeneralLisence()
  {
    String searchName = name + ":";
    int pos = GeneralLisence.indexOf(searchName);
    if (pos == -1)
    {
      // name not found in GeneralLisence, do nothing
      return;
    }
    // find the end position of the value (which is either "," or end of string)
    int endPos = GeneralLisence.indexOf(",", pos);
    if (endPos == -1)
    {
      endPos = GeneralLisence.length();
    }
    // extract the current value of the name
    String value = GeneralLisence.substring(pos + searchName.length(), endPos);
    // construct the new value based on the current state
    String newValue = (State ? "True " : "False");
    // check if the length of the old and new values are different
    if (newValue.length() != value.length())
    {
      // adjust the length by adding or removing a single comma
      if (newValue.length() > value.length())
      {
        value += ","; // add a comma to the end of the value
      }
      else
      {
        value = value.substring(0, value.length() - 1); // remove the comma at the end of the value
      }
    }
    // replace the old value with the new value
    for (int i = 0; i < newValue.length(); i++)
    {
      GeneralLisence.setCharAt(pos + searchName.length() + i, newValue.charAt(i));
    }
    // add or remove comma if necessary
    if (newValue.length() > value.length())
    {
      GeneralLisence += ",";
    }
    else if (newValue.length() < value.length())
    {
      GeneralLisence = GeneralLisence.substring(0, pos + searchName.length() + newValue.length()) + GeneralLisence.substring(endPos);
    }
  }
  void activate()
  {
    State = true;
    editGeneralLisence();
    EEPROM.writeString(E2ADD.licenseStatSave, GeneralLisence);
    EEPROM.commit();
  }
  void deactivate()
  {
    State = false;
    editGeneralLisence();
    EEPROM.writeString(E2ADD.licenseStatSave, GeneralLisence);
    EEPROM.commit();
  }
  void generateSerial()
  {
    // Combine the chip ID with the secret key
    uint64_t uid = ESP.getEfuseMac();
    char seed[32];
    snprintf(seed, sizeof(seed), "%012llX%s", uid, secretKey);

    // Generate a SHA-256 hash from the seed
    SHA256 sha256;
    uint8_t hash[32];
    sha256.update((const uint8_t *)seed, strlen(seed));
    sha256.finalize(hash, sizeof(hash));

    // Convert the hash to a registration code
    snprintf(realSerial, 17, "%02X%02X%02X%02X%02X%02X%02X%02X",
             hash[0], hash[1], hash[2], hash[3], hash[4], hash[5], hash[6], hash[7]);
  }
  bool isActive()
  {
    return State;
  }
};
lisence *GyroLicense;  // Key for Gyro
lisence *VoiceLicense; // Key For Voice
//----------------------Gyro
String GyroOriantation = "XY00";
LIS3DH myIMU; // Default constructor is I2C, addr 0x19.
uint8_t Xis, Yis;
int revX, revY;
#define AXEL_X_DEF 'X'
#define AXEL_Y_DEF 'Y'
#define AXEL_Z_DEF 'Z'
#define AXEL_REV '1'
#define AXEL_NOR '0'
void readaxels(uint8_t sensor, float *x, float *y)
{
  static String last;
#define ADD_INTERNAL_GYRO 0x19
#define ADD_MPU6050 0x68
  float tempx;
  float tempy;
  float tempz;

  if (sensor == ADD_INTERNAL_GYRO)
  {
    tempx = myIMU.readFloatAccelX();
    tempy = myIMU.readFloatAccelY();
    tempz = myIMU.readFloatAccelZ();
  }
  else if (sensor == ADD_MPU6050)
  {
    mpu.getEvent(&mpuAcc, &mpuGyro, &mpuTemp);
    tempx = mpuAcc.acceleration.x;
    tempy = mpuAcc.acceleration.y;
    tempz = mpuAcc.acceleration.z;
  }

  if (1) //(last != GyroOriantation)
  {
    last = GyroOriantation;
    Xis = GyroOriantation.charAt(0);
    Yis = GyroOriantation.charAt(1);
    if (GyroOriantation.charAt(2) == AXEL_REV)
    {
      revX = -1;
    }
    else if (GyroOriantation.charAt(2) == AXEL_NOR)
    {
      revX = 1;
    }

    if (GyroOriantation.charAt(3) == AXEL_REV)
    {
      revY = -1;
    }
    else if (GyroOriantation.charAt(3) == AXEL_NOR)
    {
      revY = 1;
    }
  }

  if (Xis == AXEL_X_DEF)
    *x = tempx;
  if (Xis == AXEL_Y_DEF)
    *x = tempy;
  if (Xis == AXEL_Z_DEF)
    *x = tempz;

  if (Yis == AXEL_X_DEF)
    *y = tempx;
  if (Yis == AXEL_Y_DEF)
    *y = tempy;
  if (Yis == AXEL_Z_DEF)
    *y = tempz;
}
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
void I2C_SENSORS_TASK(void *parameters)
{
#define CHECK_INTERVAL 500
#define BARO_INTERVAL 200
#define GYRO_INTERVAL 5
#define HUM_INTERVAL 200
#define ADD_INTERNAL_GYRO 0x19
#define ADD_INTERNAL_BAROMETER 0x5C
#define ADD_AHT10 0x38
#define ADD_MPU6050 0x68
#define ADD_BMP280 0x76
#define NO_SENSOR 0
  Wire.begin();
  uint8_t mainGyro = NO_SENSOR;
  uint8_t mainBarometer = NO_SENSOR;
  uint8_t mainHumSensor = NO_SENSOR;
  I2CDevice internalGyro(ADD_INTERNAL_GYRO, "INTERNAL_GYRO");
  I2CDevice internalBarometer(ADD_INTERNAL_BAROMETER, "INTERNAL_BAROMETER");
  I2CDevice externalAHT10(ADD_AHT10, "AHT10");
  I2CDevice externalMPU6050(ADD_MPU6050, "MPU6050");
  I2CDevice externalBMP280(ADD_BMP280, "BMP280");
  int cntr = 0;
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);
    // Cheking for Sensors connectivitys and initializing them
    if (((cntr++) % CHECK_INTERVAL) == 0)
    {
      //--------------internalBarometer
      if (internalBarometer.lastState() == false)
      {
        if (internalBarometer.isConnected() == true)
        {
          Serial.print(internalBarometer.getName() + " Connected !");
          if (BARO.begin())
          {
            Serial.println("...OK");
            mainBarometer = ADD_INTERNAL_BAROMETER;
          }
          else
          {
            Serial.println("...Failed");
          }
        }
      }
      else
      {
        if (internalBarometer.isConnected() == false)
        {
          Serial.println(internalBarometer.getName() + " Disconnected !");
          mainBarometer = NO_SENSOR;
        }
      }
      //--------------internalGyro Checking
      if (internalGyro.lastState() == false || mainGyro == NO_SENSOR)
      {
        if (internalGyro.isConnected() == true)
        {
          Serial.print(internalGyro.getName() + " Connected !");

          if (myIMU.begin() == 0) // means no error for this special libarary
          {
            Serial.println("...OK");
            mainGyro = ADD_INTERNAL_GYRO;

            myIMU.settings.accelRange = 2;
            myIMU.settings.accelSampleRate = 200;
            myIMU.applySettings();
          }
          else
          {
            Serial.println("...Failed");
          }
        }
      }
      else
      {
        if (internalGyro.isConnected() == false)
        {
          Serial.println(internalGyro.getName() + " Disconnected !");
          mainGyro = NO_SENSOR;
        }
      }
      //--------------ExternalGyro Checking
      if (externalMPU6050.lastState() == false)
      {
        if (externalMPU6050.isConnected() == true)
        {
          Serial.print(externalMPU6050.getName() + " Connected !");
          if (mpu.begin())
          {
            Serial.println("...OK");
            mainGyro = ADD_MPU6050;
          }
          else
          {
            Serial.println("...Failed");
          }
        }
      }
      else
      {
        if (externalMPU6050.isConnected() == false)
        {
          Serial.println(externalMPU6050.getName() + " Disconnected !");
          mainGyro = NO_SENSOR;
        }
      }
      //--------------External Hum Sensor selecting
      // BMP280
      if (externalBMP280.lastState() == false)
      {
        if (externalBMP280.isConnected() == true)
        {
          Serial.print(externalBMP280.getName() + " Connected !");

          if (bme.begin() == true)
          {
            Serial.println("...OK");
            mainHumSensor = ADD_BMP280;
          }
          else
          {
            Serial.println("...Failed");
          }
        }
      }
      else
      {
        if (externalBMP280.isConnected() == false)
        {
          Serial.println(externalBMP280.getName() + " Disconnected !");
          mainHumSensor = NO_SENSOR;
        }
      }
      // AHT10
      if (externalAHT10.lastState() == false)
      {
        if (externalAHT10.isConnected() == true)
        {
          Serial.print(externalAHT10.getName() + " Connected !");
          if (aht.begin() == true)
          {
            Serial.println("...OK");
            mainHumSensor = ADD_AHT10;
          }
          else
          {
            Serial.println("...Failed");
          }
        }
      }
      else
      {
        if (externalAHT10.isConnected() == false)
        {
          Serial.println(externalAHT10.getName() + " Disconnected !");
          mainHumSensor = NO_SENSOR;
        }
      }
    }
    // Actual mesurments and filtering
    if (cntr % BARO_INTERVAL == 0 && mainBarometer == ADD_INTERNAL_BAROMETER && GyroLicense->isActive() == true)
    {
      digitalAlt = psiToMeters(BARO.readPressure(PSI) - pressurCalOffset);
    }
    if (cntr % GYRO_INTERVAL == 0)
    {
      if (mainGyro == ADD_INTERNAL_GYRO) //&& GyroLicense->isActive() == true) must be checked i disabled it for ali for now
      {
        float tempx, tempy;
        readaxels(ADD_INTERNAL_GYRO, &tempx, &tempy);
        accXValue = LOW_PASS_FILTER(tempx, accXValue, sigmoid(50 * fabs(accXValue - (mpuAcc.acceleration.x / 10.0F)), 0.90, 10, 2));
        accYValue = LOW_PASS_FILTER(tempy, accYValue, sigmoid(50 * fabs(accYValue - (mpuAcc.acceleration.y / 10.0F)), 0.90, 10, 2)); // accXValue = LOW_PASS_FILTER(tempx, accXValue, sigmoid(50 * fabs(accXValue - (mpuAcc.acceleration.x / 10.0F)), 0.9999, 10, 2));
        if (GyroOffsetingFlg)
        {
          accXValue = LOW_PASS_FILTER(tempx, accXValue, 0.0);
          accYValue = LOW_PASS_FILTER(tempy, accYValue, 0.0);
          for (int i = 0; i < 500; i++)
          {
            readaxels(ADD_INTERNAL_GYRO, &tempx, &tempy);
            accXValue = LOW_PASS_FILTER(tempx, accXValue, 0.99);
            accYValue = LOW_PASS_FILTER(tempy, accYValue, 0.99);
          }
          GyroOffsetingFlg = false;
        }
      }
      if (mainGyro == ADD_MPU6050)
      {
        float tempx, tempy;
        readaxels(ADD_MPU6050, &tempx, &tempy);
        accXValue = LOW_PASS_FILTER(mpuAcc.acceleration.x / 10.0F, accXValue, 0.98);
        accYValue = LOW_PASS_FILTER(mpuAcc.acceleration.y / 10.0F, accYValue, 0.98);
      }
    }
    if (cntr % HUM_INTERVAL == 0)
    {
      if (mainHumSensor == ADD_BMP280)
      {
        digitalTemp = bme.readTemperature();
        digitalHum = bme.readHumidity();
        digitalPress = bme.readPressure();
      }
      if (mainHumSensor == ADD_AHT10)
      {
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
        digitalHum = humidity.relative_humidity;
        digitalTemp = temp.temperature;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void sendCmdToExecute(char *str);
void dimmerShortCircuitIntrupt();
void defaultCalibrations();
int dimShortFlg = false;
int dimShortNum = 0;
//=======================TEST
void onDataReceived(uint8_t *pData, size_t length);
//-------------------------------------------------TASKS
void Reg_Uptime_Task(void *parameters)
{
  if (!xrtLcns->openLog())
  {
    Serial.println("Error opening or creating Lisence file");
    vTaskDelete(NULL);
  }
  if (!xrtLizing->openLog())
  {
    Serial.println("Error opening or creating Lizing file");
    vTaskDelete(NULL);
  }
  int min_10 = 0;
  for (;;)
  {
    if (xrtLcns->isActive(xrtLcns->wrkLcns))
    {
      if (++min_10 % 600 == 0) // 10 min
      {
        xrtLizing->Uptime_tick();
        xrtLizing->saveTime(xrtLizing->uptime);
        Serial.println(xrtLizing->timeToStr(xrtLizing->uptime));
        Serial.println(xrtLizing->timeToStr(xrtLizing->expTime));
      }
      if (xrtLizing->getTime(xrtLizing->uptime) > xrtLizing->getTime(xrtLizing->expTime))
      {
        xrtLcns->deactivate(xrtLcns->wrkLcns);
      }
    }
    else
    {
      bool flag = false;
      for (int i = 1; i <= 8; i++)
      {
        if (relState_0_15(i - 1))
        {
          char str[16];
          sprintf(str, "SET_SW_%d=OFF\n", i);
          sendCmdToExecute(str);
          vTaskDelay(pdMS_TO_TICKS(100));
          flag = true;
        }
      }
      if (flag)
      {
        String str = "XrouteAlarm=No active license! Please check your license status!!\n";
        ws.sendToAll(str.c_str());
        vTaskDelay(pdTICKS_TO_MS(1000));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // for 10 min it should be 1000ms
  }
}
void ConditionsTask(void *parameters)
{
  vTaskDelay(pdMS_TO_TICKS(6000));
  Serial.println("Conditions Task Started");
  for (;;)
  {
    int vectorSize = cndtions.size();
    for (int i = 0; i < vectorSize; i++)
    {
      if (i < cndtions.size())
      {
        cndtions[i].doWork();
      }
      else
      {
        Serial.println("ConditionsTask: Vector modified, skipping invalid index.");
      }
    }
    // Serial.printf("Stack high watermark: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    //  Delay before the next iteration
    vTaskDelay(1000);
  }
}
void loadStateFromFile()
{
  // Serial.println("Loading Last States");
  //  Open the file for reading
  File f = SPIFFS.open(statesFile, FILE_READ);
  if (!f)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  // Read the contents of the file into a string
  String content = f.readString();
  f.close();
  uint16_t tmp;
  // Extract the relay states and dim values from the string
  sscanf(content.c_str(), "RELAYS:%d,D0:%f,D1:%f,D2:%f,D3:%f,D4:%f,D5:%f,D6:%f",
         &tmp, &dimTmp[0], &dimTmp[1], &dimTmp[2], &dimTmp[3], &dimTmp[4], &dimTmp[5], &dimTmp[6]);

  RELAYS.relPos = tmp;
  setRelay(RELAYS.relPos, v / 10);
  DimValChanged = true;
}
void saveStatesToFile()
{
  // Buffer to store the state string
  char str[256];
  // Form the state string
  snprintf(str, sizeof(str), "RELAYS:%d,D0:%f,D1:%f,D2:%f,D3:%f,D4:%f,D5:%f,D6:%f",
           RELAYS.relPos, dimTmp[0], dimTmp[1], dimTmp[2], dimTmp[3], dimTmp[4], dimTmp[5], dimTmp[6]);
  // Attempt to open the file for writing
  File f = SPIFFS.open(statesFile, FILE_WRITE);
  if (!f)
  {
    // If file opening fails, handle error (e.g., log error or proceed)
    Serial.println("Failed to open file for writing in resetHandler");
  }
  else
  {
    // Write the state string to the file
    if (f.write((uint8_t *)str, strlen(str)) != strlen(str))
    {
      // If writing fails, handle error
      Serial.println("Failed to write complete state string in resetHandler");
    }
    // Close the file
    f.close();
  }
}
bool MeasurmentTaskPause = false;
void MeasurmentTask(void *parameters)
{
  char str[64];
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);
    while (MeasurmentTaskPause)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Set relay voltages acording to powersupply
    if (relaySatat() == REL_FREE)
    {
      setRelPWM(REL_HOLD_VOLTAGE, v / 10);
    }
    /* -----------reading and filtering is done in adcReadingTask*/
    a0 = ((amp0 - amp0Offset) * A0calCo);
    if (ampSenisConnected)
    {
      a1 = ((amp1 - amp1Offset) * A1calCo);
    }
    else
    {
      a1 = 0;
    }
    a2 = ((amp2 - amp2Offset) * A2calCo);
    v = (volt * VcalCo) - (cableResistance * a1 / 100.0F) - (Negvolt * 10.0F);
    w = a1 / 10.0F * v / 10.0F;
    // b is now being calculated in the battery monitoring task
    // b = (v - battEmptyVoltage) / (battFullVoltage - battEmptyVoltage) * 100;
    // b = constrain(b, 0, 100);
    if ((a1 + a2) < 0)
      battHourLeft = LOW_PASS_FILTER((b * batteryCap) / fabs(a1 + a2), battHourLeft, 0.98);
    else
      battHourLeft = 0;
    cwPrcnt = (clnWtr - clnWtrMin) / (clnWtrMax - clnWtrMin) * 1000;
    dwPrcnt = (drtWtr - drtWtrMin) / (drtWtrMax - drtWtrMin) * 1000;
    gwPrcnt = (gryWtr - gryWtrMin) / (gryWtrMax - gryWtrMin) * 1000;
    cwPrcnt = constrain(cwPrcnt, 0, 1820);
    dwPrcnt = constrain(dwPrcnt, 0, 1820);
    gwPrcnt = constrain(gwPrcnt, 0, 1820);
    pt100mv = pt100 * PT_mvCal;
    String data = "";
    //=====Gyro calculation
    if (true) //(GyroLicense->isActive()) // it must be checked later , i disabled lisence for ali cheching
    {
      float ofsetlesX = (accXValue - accXValueOffset) * revX;
      float ofsetlesY = (accYValue - accYValueOffset) * revY;
      // print
      degX = asin(ofsetlesX) * 180 / PI;
      degY = asin(ofsetlesY) * 180 / PI;
      // handle nan
      if (isnan(degX))
        degX = 90;
      if (isnan(degY))
        degY = 90;
      // Serial.println("dX:" + String(degX, 3) + ",dY:" + String(degY, 3));
      // Serial.println("X:" + String(ofsetlesX, 3) + ",Y:" + String(ofsetlesY, 3));
      alpha = atan(ofsetlesY / ofsetlesX);
      if (ofsetlesX < 0 && ofsetlesY > 0)
        alpha += PI;
      if (ofsetlesX < 0 && ofsetlesY < 0)
        alpha += PI;
      if (ofsetlesX > 0 && ofsetlesY < 0)
        alpha += (2 * PI);
      len = sqrt(ofsetlesX * ofsetlesX + ofsetlesY * ofsetlesY) * 12; // sensitivity is 12 times now
      if (len > 1)
        len = 1;
    }
    //=====
    // sprintf(str, "SOLVOL1=%d\n", (int)v);
    // data += str;
    // sprintf(str, "CARVOL1=%d\n", (int)v);
    // data += str;
    sprintf(str, "BATVOL1=%d\n", (int)v);
    data += str;
    sprintf(str, "BATPR1=%d\n", (int)b);
    data += str;
    sprintf(str, "AMPINT1=%.2f\n", (float)a0 / 10.0F);
    data += str;
    sprintf(str, "AMPEXT1=%d\n", (int)a1);
    data += str;
    sprintf(str, "AMPEXT2=%d\n", (int)a2);
    data += str;
    sprintf(str, "WAT1=%d\n", (int)w);
    data += str;
    sprintf(str, "FLT1=%d\n", ((int)cwPrcnt) / 10 * 10);
    data += str;
    sprintf(str, "FLT2=%d\n", ((int)dwPrcnt) / 10 * 10);
    data += str;
    sprintf(str, "TMPA1=%d\n", (int)(10 * ReadPT100_Temp(pt100mv, 510))); // PT100
    data += str;
    sprintf(str, "TMPD1=%d\n", ((int)(digitalTemp * 100)) / 10);
    data += str;
    sprintf(str, "HUMD1=%d\n", (int)digitalHum);
    data += str;
    sprintf(str, "ALT1=%d\n", (int)digitalAlt);
    data += str;
    sprintf(str, "BATHUR=%d\n", (int)battHourLeft / 10);
    data += str;
    sprintf(str, "GAZ1=%d\n", (int)a1);
    data += str;
    sprintf(str, "Gyro=%1.3f,%1.3f,%1.1f,%1.1f\n", len * cos(alpha), len * sin(alpha), degX, degY);
    data += str;
    // Serial.println(String(">X:" + String(len * cos(alpha), 3) + ",Y:" + String(len * sin(alpha), 3)));
    data += "RELS=" + getRelsStatStr() + "\n";
    data += "UI_INDEX=" + String(uiConfigIndex) + "\n";
    // myBle.sendString(data);
    ws.sendToAll(data.c_str());
    // Serial.println(data);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
// websocket OTA
#define OTA_BUFFER_SIZE 2048
uint8_t *UDP_buffer = nullptr;
bool UDP_bufferBussy = true;
bool UDP_dataReady = false;
int UDP_buffLen;
int UDP_len;
TaskHandle_t I2C_SENSORS_TASK_Handle;
TaskHandle_t Reg_Uptime_Task_Handle;
TaskHandle_t ConditionsTask_Handle;
TaskHandle_t MeasurmentTaskHandle;
TaskHandle_t updateTaskHandle;
TaskHandle_t sendUiConfigTaskHandle;
TaskHandle_t sendConditionsTaskHandle;
TaskHandle_t DimerTask_Handle;
TaskHandle_t adcReadingTask_Handle;
TaskHandle_t led_indicator_task_Handle;
TaskHandle_t OVR_CRNT_PRTCT_TASK_Handle;
TaskHandle_t BatteryTask_Handle;
void pauseUnnessesaryTasks()
{
  TaskHandle_t th[9] =
      {MeasurmentTaskHandle,
       DimerTask_Handle,
       adcReadingTask_Handle,
       // led_indicator_task_Handle,
       OVR_CRNT_PRTCT_TASK_Handle,
       I2C_SENSORS_TASK_Handle,
       BatteryTask_Handle,
       ConditionsTask_Handle,
       Reg_Uptime_Task_Handle};
  for (int i = 0; i < 9; i++)
  {
    if (th[i] != NULL)
    {
      vTaskSuspend(th[i]);
    }
  }
}
void resumeUnnessesaryTasks()
{
  TaskHandle_t th[9] =
      {MeasurmentTaskHandle,
       DimerTask_Handle,
       adcReadingTask_Handle,
       // led_indicator_task_Handle,
       OVR_CRNT_PRTCT_TASK_Handle,
       I2C_SENSORS_TASK_Handle,
       BatteryTask_Handle,
       ConditionsTask_Handle,
       Reg_Uptime_Task_Handle};
  for (int i = 0; i < 9; i++)
  {
    if (th[i] != NULL)
    {
      vTaskResume(th[i]);
    }
  }
}
void updateTask(void *parameters)
{
  UDP_buffer = (uint8_t *)malloc(OTA_BUFFER_SIZE);
  UDP_bufferBussy = false;
  Update.begin(UDP_len);
  float progressPercent = 0;
  float lastProgressPercent = 0;
  uint32_t progress;
  uint64_t start = millis();
  uint TIME_OUT = 15000;
  for (;;)
  {
    while (UDP_dataReady == false)
    {
      if (millis() - start > TIME_OUT)
        break;
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (millis() - start > TIME_OUT)
    {
      Serial.println("UPD_TIMEOUT");
      ws.sendToThisClient("UPD_FAILED\n");
      Serial.println("UPD_FAILED");
      Update.end(true);
      ws.endUpdate();
      free(UDP_buffer);
      UDP_bufferBussy = false;
      // MeasurmentTaskPause = false;
      // vTaskResume(MeasurmentTaskHandle);
      resumeUnnessesaryTasks();
      vTaskDelete(NULL);
    }
    UDP_dataReady = false;
    Update.write(UDP_buffer, UDP_buffLen);
    UDP_bufferBussy = false;
    progress = Update.progress();
    progressPercent = (float)progress / UDP_len * 100;
    if ((progressPercent - lastProgressPercent) > 1)
    {
      lastProgressPercent = progressPercent;
      ws.sendToThisClient(String("UPD_PRC_1=" + String(progressPercent, 1) + "\n").c_str());
      start = millis(); // reset timer
    }
    Serial.println(String(progressPercent, 1));
    if (progress == UDP_len)
    {
      bool state = Update.end();
      if (state == true)
      {
        ws.sendToThisClient("UPD_COMPLETED\n");
        Serial.println("UPD_COMPLETED\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP.restart();
      }
      else
      {
        ws.sendToThisClient("UPD_FAILED\n");
        Serial.println("UPD_FAILED\n");
        ws.endUpdate();
        free(UDP_buffer);
        UDP_bufferBussy = false;
        // MeasurmentTaskPause = false;
        // vTaskResume(MeasurmentTaskHandle);
        resumeUnnessesaryTasks();
        vTaskDelete(NULL);
      }
    }
  }
}
void sendUiConfigTask(void *parameters)
{
  vTaskSuspend(MeasurmentTaskHandle);
  Serial.println("👁️ 👁️ SENDING UI CONFIG (via streamFile method)...");
  AsyncWebSocketClient *client = ws.getClient(); // Get the target client

  if (client)
  {
    // Call your new, safe, and efficient streaming method
    ws.streamFile(ConfigFile.c_str(), client, jsonKeys::UI_CONFIG);
  }
  else
  {
    Serial.println("No client to send UI config to.");
  }

  vTaskResume(MeasurmentTaskHandle);
  vTaskDelete(NULL);
}
void sendConditionsTask(void *parameters)
{
  vTaskSuspend(MeasurmentTaskHandle);
  String str = readStringFromFile(CondFile);
  wrapJson(str.c_str(), jsonKeys::CONDITON_CONFIG, str);
  Serial.println("SENDING CONDITON CONFIG");
  ws.sendToAll(str.c_str());
  // myBle.sendString(str.c_str());
  vTaskResume(MeasurmentTaskHandle);
  vTaskDelete(NULL);
}
void DimerTask(void *parameters)
{
  bool firstRun = true;
  vTaskDelay(1000);
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);

    if (DimValChanged == true)
    {
      int sameVal = 0;
      for (int i = 0; i < 7; i++)
      {
        dimLPF[i] = RAMPIT(dimTmp[i], dimLPF[i], absf(dimTmp[i] - dimLPF[i]) * 0.05);
        ledcWrite(channelTable[i], dimLPF[i]);
        if (absf(dimTmp[i] - dimLPF[i]) < 1)
          sameVal++;
      }
      if (sameVal == 7)
      {
        if (DimValChanged)
        {
          DimValChanged = false;
          if (!firstRun)
          {
            saveStatesToFile();
          }
          else
          {
            firstRun = false;
          }
        }
      }
      vTaskDelay(10 / portTICK_RATE_MS);
    }
    else
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}
void adcReadingTask(void *parameters)
{
  int cntr = 0;
  float lastPT100 = 0;

  negv = ADC_LPF(NEG_VOLT_MUX_IN, 1, negv, 0);
  volt = ADC_LPF(VOLT_MUX_IN, 1, volt, 0.0);
  amp0 = ADC_LPF(AMPER0_MUX_IN, 1, amp0, 0.0);
  amp1 = ADC_LPF(AMPER_MUX_IN, 1, amp1, 0.0);
  amp2 = ADC_LPF(AIR_QUALITY_MUX_IN, 1, amp2, 0.0);
  clnWtr = ADC_LPF(GAUGE1_MUX_IN, 1, clnWtr, 0.0);
  drtWtr = ADC_LPF(GAUGE2_MUX_IN, 1, drtWtr, 0.0);
  gryWtr = ADC_LPF(GAUGE3_MUX_IN, 1, gryWtr, 0.0);

  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);

    negv = ADC_LPF(NEG_VOLT_MUX_IN, 5, negv, 0.995);
    Negvolt = ((((negv - NegVoltOffset) * 5 * 1.25) / 4096) - ((24 / 124) * 3.3)) / (1 + (24 / 124)); // 1.28 is for calibration Black PCB china
    volt = ADC_LPF(VOLT_MUX_IN, 5, volt, 0.995);
    if (volt < 900) // 9.0 Volt is low for relays to work properly
    {
      lowVoltageFlg = true;
    }
    else
    {
      lowVoltageFlg = false;
    }

    amp0 = ADC_LPF(AMPER0_MUX_IN, 15, amp0, 0.995);
    amp1 = ADC_LPF(AMPER_MUX_IN, 15, amp1, 0.995);
    amp2 = ADC_LPF(AIR_QUALITY_MUX_IN, 5, amp2, 0.995);
    clnWtr = ADC_LPF(GAUGE1_MUX_IN, 15, clnWtr, 0.995);
    drtWtr = ADC_LPF(GAUGE2_MUX_IN, 15, drtWtr, 0.995);
    gryWtr = ADC_LPF(GAUGE3_MUX_IN, 15, gryWtr, 0.995);
    float val = ADC_LPF(PT100_MUX_IN, 15, pt100, 0.99);
    lastPT100 = LOW_PASS_FILTER(pt100, lastPT100, 0.99);
    pt100 = ADC_LPF(PT100_MUX_IN, 15, pt100, sigmoid(fabs(lastPT100 - val), 0.9999, 1.7, 20));
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void led_indicator_task(void *parameters)
{
  for (;;)
  {
    if (UpdatingFlg)
    {
      strip.setBrightness(250);
    }
    while (UpdatingFlg)
    {
      ws2812Blink(COLOR_WHITE, 1, 3, 1);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    if (overCrntFlg == true)
    {
      ws2812Blink(COLOR_ORANG, 1, 3, 1);
    }
    // TODO : add wifi connection later on for color RED and BLUE
    int clients = ws.clientCount();
    if (clients > 0)
    {
      if (ws.isUpdating())
        ws2812Blink(COLOR_WHITE, clients, 1, 0.8);
      else
      {
        ws2812Blink(COLOR_GREEN, clients, 3, 0.5);
        ws2812Blink(COLOR_RED, 1, 4, 0.5);
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
void OVR_CRNT_PRTCT_TASK(void *parameters)
{
#define DECRS_STP_SIZE 100
  float a0Fast;
  int val;
  char str[128];
  int cntr = 0;
  vTaskDelay(5000 / portTICK_PERIOD_MS); // waiting for measurments to stablize
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);
    if (dimShortFlg)
    {
      ledcWrite(channelTable[dimShortNum], 0);
      sprintf(str, "DIMER%d=0\n", dimShortNum + 1);
      sendToAll(str);
      sprintf(str, "XrouteAlarm= PROTECTION! Over current at DIMMER : %d\n", dimShortNum + 1);
      sendToAll(str);
      vTaskDelay(1000);
      dimShortFlg = false;
    }
    a0Fast = ((amp0 - amp0Offset) * A0calCo);
    if (0) // disable overCurrent protection
      if (abs(a0Fast) > 140)
      {
        overCrntFlg = true;
        if (!overCrntFlg)
        {
          sprintf(str, "XrouteAlarm=OVER CURRENT protection. Amp > 14.0\n");
          sendToAll(str);
        }
        for (int i = 0; i < 7; i++)
        {
          if (dimTmp[i] > DECRS_STP_SIZE)
          {
            dimTmp[i] = 0; // dimTmp[i] - DECRS_STP_SIZE;
            DimValChanged = true;
          }
          val = dimTmp[i] * 255 * dimLimit[i] / 32768;
        }
      }
    if (overCrntFlg)
    {
      if (cntr++ > 2000)
      {
        overCrntFlg = false;
        cntr = 0;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void cpuMonitoringTask(void *parameters)
{
  cpu_monitor.begin(2000, 1000);
  vTaskDelay(1000);
  for (;;)
  {
    float usage_core0 = cpu_monitor.get_cpu_usage_core0();
    float usage_core1 = cpu_monitor.get_cpu_usage_core1();
    float usage_total = cpu_monitor.get_cpu_usage_total();
    Serial.printf("Core 0: %.1f%% | Core 1: %.1f%% | Average: %.1f%%\n",
                  usage_core0, usage_core1, usage_total);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
void ramMonitorTask(void *pvParameters)
{
  (void)pvParameters; // Unused parameter
  for (;;)
  {
    // Measure RAM usage
    size_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t ramUsed = totalHeap - freeHeap;

    // Print the RAM usage in KB
    Serial.print("RAM Used: ");
    Serial.print(ramUsed / 1024);
    Serial.print(" KB (");

    // Calculate the RAM usage in percentage
    float ramUsedPercent = (static_cast<float>(ramUsed) / totalHeap) * 100.0;
    Serial.print(ramUsedPercent, 2); // Display with 2 decimal places
    Serial.print("%)");

    // Print the total RAM size in KB
    Serial.print("\tTotal RAM Size: ");
    Serial.print(totalHeap / 1024);
    Serial.println(" KB");

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
  }
}
void BatteryTask(void *parameters)
{
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  if (battFullVoltage < 180)
  {
    myBattery.init(BATTERY_TYPE_AGM, batteryCap, BATTERY_CONFIG_12V, v);
  }
  else
  {
    myBattery.init(BATTERY_TYPE_AGM, batteryCap, BATTERY_CONFIG_24V, v);
  }
  String str = myBattery.SelectBatteryAcordingToFullVoltage((int)battFullVoltage);
  Serial.println("Battery init: " + str);
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);

    myBattery.loop(v, a1, (v - battEmptyVoltage) / (battFullVoltage - battEmptyVoltage));

    // it means eather sensor is connected or charging is lokily is under +40Amps. so need to decet sensor some how else later
    if (amp1 > 200)
    {
      if (!ampSenisConnected)
      {
        sendToAll("XrouteAlarm=Amp Meter Connected !\n");
        ampSenisConnected = true;
        myBattery.setPercent(myBattery.getBtPerV());
      }
      b = myBattery.getPercent() * 100;
    }
    else
    {
      if (ampSenisConnected)
      {
        sendToAll("XrouteAlarm=Amp Meter Disconnected !\n");
        ampSenisConnected = false;
      }
      b = constrain(myBattery.btPerV, 0, 1) * 100;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void defaultCalibrations()
{
  char str[128];

  EEPROM.writeFloat(E2ADD.lowVoltageSave, lowVoltageDeflt);
  EEPROM.writeInt(E2ADD.lowVoltageRelaysSave, lowVoltageRelaysDeflt);
  EEPROM.writeInt(E2ADD.lowVoltageDimersSave, lowVoltageDimersDeflt);
  EEPROM.writeFloat(E2ADD.criticalVoltageSave, criticalVoltageDeflt);
  EEPROM.writeInt(E2ADD.criticalVoltageRelaysSave, criticalVoltageRelaysDeflt);
  EEPROM.writeInt(E2ADD.criticalVoltageDimersSave, criticalVoltageDimersDeflt);
  EEPROM.writeUInt(E2ADD.E2promFirsTime, E2PROM_NOT_FIRST_TIME_RUN_VAL);
  EEPROM.writeUInt(E2ADD.blePassSave, blePassDeflt);
  EEPROM.writeFloat(E2ADD.pressurCalOffsetSave, pressurCalOffsetDeflt);
  EEPROM.writeFloat(E2ADD.accXValueOffsetSave, accXValueOffsetDeflt);
  EEPROM.writeFloat(E2ADD.accYValueOffsetSave, accYValueOffsetDeflt);
  EEPROM.writeFloat(E2ADD.VcalCoSave, VcalCoDeflt);
  EEPROM.writeFloat(E2ADD.NegVoltOffsetSave, NegVoltOffsetDeflt);
  EEPROM.writeFloat(E2ADD.amp0OffsetSave, amp0OffsetDeflt);
  EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCoDeflt);
  EEPROM.writeFloat(E2ADD.ampOffsetSave, ampOffsetDeflt);
  EEPROM.writeFloat(E2ADD.AcalCoSave, AcalCoDeflt);
  EEPROM.writeFloat(E2ADD.amp2OffsetSave, amp2OffsetDeflt);
  EEPROM.writeFloat(E2ADD.PT_mvCal_Save, PT_mvCal_Deflt);
  EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCoDeflt);
  EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMinDeflt);
  EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMaxDeflt);
  EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMinDeflt);
  EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMaxDeflt);
  EEPROM.writeFloat(E2ADD.gryWtrMinSave, gryWtrMinDeflt);
  EEPROM.writeFloat(E2ADD.gryWtrMaxSave, gryWtrMaxDeflt);
  EEPROM.writeFloat(E2ADD.batteryCapSave, batteryCapDeflt); // 100AH must be defafault
  EEPROM.writeFloat(E2ADD.battFullVoltageSave, battFullVoltageDeflt);
  EEPROM.writeFloat(E2ADD.battEmptyVoltageSave, battEmptyVoltageDeflt);
  EEPROM.writeFloat(E2ADD.cableResistanceSave, cableResistanceDeflt);
  for (int i = 0; i < 7; i++) // save dimmer defaults
  {
    EEPROM.writeFloat(E2ADD.dimLimitSave[i], dimLimitDeflt); // limit dimers to %60
  }
  if (EEPROM.commit())
  {
    sprintf(str, "XrouteAlarm=Default Saved OK !\n");
    sendToAll(str);
    loadSavedValue();
    for (int i = 0; i < 7; i++) // load dimmer defaults
    {
      dimLimit[i] = EEPROM.readFloat(E2ADD.dimLimitSave[i]);
    }
    for (int i = 0; i < 7; i++) // show dimmer defaults
    {
      float val = dimLimit[i] * 128;
      sprintf(str, "dimMax%d.val=%d\n", i + 1, (int)val);
      sendToAll(str);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
  else
  {
    sprintf(str, "XrouteAlarm=SAVE ERROR !\n");
    sendToAll(str);
  }
}
// END----------------------------------------------TASKS
//-------------------------------------------------FUNCTIONS
void createCondition(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue)
{
  cndtions.push_back(Conditions(_inputType, _inputPort, _oprt, _setpoint, _outputType, _outputPort, _outputValue)); // 0
}
void createTimerCondition(String _outputType, int _motorPort, int _upTimeMs, int downTimeMs)
{
  cndtions.push_back(Conditions(_outputType, _motorPort, _upTimeMs, downTimeMs)); // 0
}
//_______________________PARSER
void processReceivedCommandData(uint8_t *pData, size_t length)
{
  char str[128];
  static RGB_VALS RGB1;
  static String accumulatedData;
  String receivedData(reinterpret_cast<char *>(pData), length);
  accumulatedData += receivedData;
  int endPos;
  while ((endPos = accumulatedData.indexOf('\n')) != -1)
  {
    MeasurmentTaskPause = true;
    String command = accumulatedData.substring(0, endPos);
    // remove \r if exist at the end beqause in postman it sends \n\r
    if (command.endsWith("\r"))
    {
      command.remove(command.length() - 1);
    }
    accumulatedData.remove(0, endPos + 1);
    int RES = 0;
    /**/ if (command.startsWith("SET_"))
    {
      command = command.substring(4);
      /**/ if (command.startsWith("DIM_")) // DIM_12=34
      {
        int dimNumber = command.substring(command.indexOf("_") + 1, command.indexOf("=")).toInt() - 1;
        if (dimNumber > 1 || dimNumber <= MyBoard.dimCount) // todo : it was (dimNumber<6) and im not sure about this walue now
        {
          String valStr = command.substring(command.indexOf("=") + 1);
          float val = valStr.toFloat() / 255;
          dimTmp[dimNumber] = 32768 * val * dimLimit[dimNumber];
          DimValChanged = true;
          sprintf(str, "DIMER%d=%s\n", dimNumber + 1, valStr);
          // myBle.sendString(str);
          //   ws.sendToAll(str);
          ws.SendToAllExcludeClient(str, ws.getClient()); // send to all cliants except the one who is ben this command received from
          // Serial.printf("in:%s | out:%s\n", command, str);
        }
        else
        {
          RES = 1;
        }
      }
      else if (command.startsWith("RGB_")) // SET_RGB_1=128,125,23,255; => RED,GREEN,BLUE,BRIGHTNESS;
      {
        int index = atoi(command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).c_str());
        // extract and store in RGB1
        int start = command.indexOf("=") + 1;
        int comma1 = command.indexOf(",", start);
        int comma2 = command.indexOf(",", comma1 + 1);
        int comma3 = command.indexOf(",", comma2 + 1);
        if (index == 1)
        {
          RGB1.red = command.substring(start, comma1).toInt();
          RGB1.green = command.substring(comma1 + 1, comma2).toInt();
          RGB1.blue = command.substring(comma2 + 1, comma3).toInt();
          RGB1.brightness = command.substring(comma3 + 1).toInt();
        }
      }
      else if (command.startsWith("SW_")) // SW_12=ON
      {
        int index = atoi(command.substring(command.indexOf("_") + 1, command.indexOf('=')).c_str());
        if (command.lastIndexOf("ON") > 0)
        {
          RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[index - 1]);
          setRelay(RELAYS.relPos, v / 10);
          ws.SendToAllExcludeClient(String("sw" + String(index) + "=ON\n").c_str(), ws.getClient());
        }
        else if (command.lastIndexOf("OFF") > 0)
        {
          RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[index - 1]);
          setRelay(RELAYS.relPos, v / 10);
          ws.SendToAllExcludeClient(String("sw" + String(index) + "=OFF\n").c_str(), ws.getClient());
        }
        else
        {
          RES = 1;
        }
        // saveStatesToFile(); to do add save again . its for preventing damage to flash for test
      }
      else if (command.startsWith("MAX_DIM_")) // MAX_DIM_
      {
        int dimNumber = command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).toInt() - 1;
        if (dimNumber > 1 || dimNumber < 6) // handling unwanted value from app
        {
          String valStr = command.substring(command.indexOf("=") + 1);
          float val = valStr.toFloat() / 255;
          dimTmp[dimNumber] = 32768 * val * 1; // dimLimit[dimNumber];
          DimValChanged = true;
          char str[128];
          sprintf(str, "DIMER%d=%s\n", dimNumber + 1, valStr);
          // myBle.sendString(str);
          //  ws.sendToAll(str);
          ws.SendToAllExcludeClient(str, ws.getClient()); // send to all cliants except the one who is ben this command received from
          // Serial.printf("in:%s | out:%s\n", command, str);
        }
        else
        {
          RES = 1;
        }
      }
      else if (command.startsWith("MOT_")) // MOT_1=UP
      {
        int index = atoi(command.substring(command.indexOf("_") + 1, command.indexOf('=')).c_str());
        /* */ if (command.indexOf("UP") > 0)
        {
          motorWay = MOTOR_UP;
          RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[7 - 1]);
          RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[8 - 1]);
          setRelay(RELAYS.relPos, v / 10);
          // myBle.sendString("Motor1=Up\n");
          ws.sendToAll(String("Motor1=Up\n").c_str());
        }
        else if (command.indexOf("DOWN") > 0)
        {
          motorWay = MOTOR_DOWN;
          RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[8 - 1]);
          RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[7 - 1]);
          setRelay(RELAYS.relPos, v / 10);
          // myBle.sendString("Motor1=Down\n");
          ws.sendToAll(String("Motor1=Down\n").c_str());
        }
        else if (command.indexOf("STOP") > 0)
        {
          motorWay = MOTOR_STOP;
          RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[7 - 1]);
          RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[8 - 1]);
          setRelay(RELAYS.relPos, v / 10);
          // myBle.sendString("Motor1=Stop\n");
          ws.sendToAll(String("Motor1=Stop\n").c_str());
        }
      }
      else if (command.startsWith("ZERO_GYRO_")) // Zero Gyro
      {
        uint timeout = 200;
        GyroOffsetingFlg = true;
        while (GyroOffsetingFlg && --timeout > 0)
          vTaskDelay(10 / portTICK_PERIOD_MS);
        EEPROM.writeFloat(E2ADD.accXValueOffsetSave, accXValue);
        EEPROM.writeFloat(E2ADD.accYValueOffsetSave, accYValue);
        EEPROM.commit();
        accXValueOffset = EEPROM.readFloat(E2ADD.accXValueOffsetSave);
        accYValueOffset = EEPROM.readFloat(E2ADD.accYValueOffsetSave);
        if (timeout)
        {
          sprintf(str, "XrouteAlarm=accXValueOffset=%f,accYValueOffset=%f\n", accXValueOffset, accYValueOffset);
        }
        else
        {
          sprintf(str, "XrouteAlarm=Try again please!\n");
        }
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("START_UPDATE_")) // START_UPDATE_1=5789 byte
      {
        // MeasurmentTaskPause = true;
        // vTaskSuspend(MeasurmentTaskHandle);
        pauseUnnessesaryTasks();
        int index = command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).toInt() - 1;
        uint32_t binSize = command.substring(command.indexOf("=") + 1).toInt();
        ws.startUpdate(binSize);
        UDP_len = binSize;
        // start updateTask
        if (eTaskGetState(&updateTaskHandle) != eRunning)
          xTaskCreate(updateTask, "updateTask", 4 * 1024, NULL, 3, &updateTaskHandle);
      }
      else if (command.startsWith("STOP_UPDATE_")) // STOP_UPDATE_1
      {
        // do something to hult the update and send back update if failed later : to do
      }
      else if (command.startsWith("DEV_RESET_1")) // Restart xRoute
      {
        Serial.println("[PARSER]:Restarting ESP32");
        // pauseUnnessesaryTasks();
        vTaskSuspend(led_indicator_task_Handle);
        ws2812Blink(COLOR_WHITE, 10, 1, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP.restart();
      }
      else if (command.startsWith("DEL_CORE_DUMP_")) // delete coredump
      {
        coreDump->delete_core_dump();
      }
      else
      {
        RES = 1;
      }
    }
    else if (command.startsWith("GET_"))
    {
      command = command.substring(4);
      /* */ if (command.startsWith("INIT_")) // GET_INIT
      {
        int index = atoi(command.substring(command.lastIndexOf("_") + 1).c_str());

        String str = "";
        for (int index = 1; index <= MyBoard.relCount; index++)
        {
          if (relState_0_15(index - 1) == true)
          {
            str += "sw" + String(index) + "=ON\n";
          }
          else
          {
            str += "sw" + String(index) + "=OFF\n";
          }
        }
        for (int i = 1; i <= MyBoard.dimCount; i++)
        {
          float val = (dimTmp[i - 1] / (32768 * dimLimit[i - 1])) * 255;
          str += "DIMER" + String(i) + "=" + String((int)val) + "\n";
        }
        // myBle.sendString(str);
        ws.sendToAll(str.c_str());
      }
      else if (command.startsWith("DIM_LIM_")) // GET_DIM_LIM_1
      {
        int index = atoi(command.substring(command.lastIndexOf("_") + 1).c_str());

        String str;
        for (int i = 0; i < 5; i++)
        {
          float val = dimLimit[i] * 255;
          str += "DIM_LIM" + String(i + 1) + "=" + String((int)val) + "\n";
        }
        // myBle.sendString(str);
        ws.sendToAll(str.c_str());
      }
      else if (command.startsWith("RGB_")) // GET_RGB_1
      {
        ws.sendToAll(String("RGB_INFO_1=" + String(RGB1.red) + "," + String(RGB1.green) + "," + String(RGB1.blue) + "," + String(RGB1.brightness) + "\n").c_str());
      }
      else if (command.startsWith("VER_"))
      {
        String response = "Version=" + Version + "\n";
        // Serial.println(response.c_str());
        // myBle.sendString(response.c_str());
        ws.sendToThisClient(response.c_str());
        // Serial.println("🆚" + String(response.c_str()));
      }
      else if (command.startsWith("CONDITION_CONFIG"))
      {
        if (eTaskGetState(&sendConditionsTaskHandle) != eRunning)
        {
          xTaskCreate(sendConditionsTask, "sendConditionsTask", 1024 * 10, NULL, 2, &sendConditionsTaskHandle);
        }
      }
      else if (command.startsWith("UI_CONFIG"))
      {
        if (eTaskGetState(&sendUiConfigTaskHandle) != eRunning)
        {
          xTaskCreate(sendUiConfigTask, "sendUiConfigTask", 1024 * 4, NULL, 2, &sendUiConfigTaskHandle); // TODO : i need streaming otherwise it will crash on big jsons
        }
      }
      else if (command.startsWith("GYRO_ORI_")) // gyro oriantation
      {
        String response = "Orientation=" + GyroOriantation + "\n";
        // myBle.sendString(response.c_str());
        ws.sendToThisClient(response.c_str());
      }
      else if (command.startsWith("DIM_LIM_")) // dimer limit DIM_LIM_123
      {
        int index = atoi(command.substring(command.indexOf("_") + 1).c_str());
        float val = dimLimit[index] * 255;
        sprintf(str, "dimMax%d.val=%d\n", index + 1, (int)val);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("WIFI_INFO_JSON_"))
      {
        int index = (command.substring(command.lastIndexOf("_") + 1)).toInt();
        String msg = "{ \"WIFI_INFO\" : [{";
        msg += "\"ssid\" : \"" + wifi_WebSocket_Settings.get<String>(NetworkKeys::WifiSSID) + "\",";
        msg += "\"pass\" : \"" + wifi_WebSocket_Settings.get<String>(NetworkKeys::WifiPassword) + "\",";
        msg += "\"apName\" : \"" + wifi_WebSocket_Settings.get<String>(NetworkKeys::ApName) + "\",";
        msg += "\"apPass\" : \"" + wifi_WebSocket_Settings.get<String>(NetworkKeys::ApPassword) + "\",";
        msg += "\"hostN\" : \"" + wifi_WebSocket_Settings.get<String>(NetworkKeys::HostName) + "\",";
        msg += "\"STA_AP\" : " + String(wifi_WebSocket_Settings.get<int>(NetworkKeys::STA_AP)) + ",";
        msg += "\"LOCAL_IP\" : \"" + WiFi.localIP().toString() + "\",";
        msg += "\"RSSI\" : " + String(WiFi.RSSI()) + ",";
        msg += "\"LAST_ATEMP\" : " + String(wifi_WebSocket_Settings.get<int>(NetworkKeys::AtempResault)) + ",";
        msg += "\"port\" : " + String(wifi_WebSocket_Settings.get<int>(NetworkKeys::Port)) + "}]}";
        ws.sendToThisClient(msg.c_str());
      }
      else if (command.startsWith("DEVICE_INFO_JSON_"))
      {
        uint64_t chipid = ESP.getEfuseMac();
        char UIDstr[16];
        sprintf(UIDstr, "%012llX", chipid);
        int index = (command.substring(command.lastIndexOf("_") + 1)).toInt();
        StaticJsonDocument<1024> doc;

        JsonObject root = doc.createNestedObject("DEVICE_INFO");
        JsonObject licenses = root.createNestedObject("licenses");
        licenses["Gyro"] = xrtLcns->isActive(xrtLcns->gyroLcns) ? "True" : "False";
        licenses["Humidity"] = xrtLcns->isActive(xrtLcns->humLcns) ? "True" : "False";
        licenses["Current"] = xrtLcns->isActive(xrtLcns->crntLcns) ? "True" : "False";
        licenses["Work"] = xrtLcns->isActive(xrtLcns->wrkLcns) ? "True" : "False";
        licenses["Gas"] = xrtLcns->isActive(xrtLcns->gasLcns) ? "True" : "False";

        JsonObject info = root.createNestedObject("info");
        info["MAC_ADDRESS"] = UIDstr;
        info["DEVICE_NAME"] = "xRoute-pro";
        info["VERSION"] = Version;
        info["REL_COUNT"] = MyBoard.relCount;
        info["DIM_COUNT"] = MyBoard.dimCount;
        info["FLT_COUNT"] = MyBoard.fltCount;
        info["MOTOR_COUNT"] = MyBoard.motorCount;
        info["BOARD_TYPE"] = MyBoard.Name;

        String msg;
        serializeJson(doc, msg);
        // Serial.println(msg);
        ws.sendToThisClient(msg.c_str());
      }
      else if (command.startsWith("WIFI_IP_"))
      {
        int index = (command.substring(command.lastIndexOf("_") + 1)).toInt(); // unused for now
        String myIp = "WIFI_IP_1=" + WiFi.localIP().toString() + "\n";
        ws.sendToThisClient(myIp.c_str());
      }
      else if (command.startsWith("HOST_NAME_"))
      {
        int index = command.substring(command.lastIndexOf("_") + 1).toInt();
        String hostName = wifi_WebSocket_Settings.get<String>(NetworkKeys::HostName);
        hostName = "HOST_NAME_1=" + hostName + "\n";
        ws.sendToThisClient(hostName.c_str());
      }
      else if (command.startsWith("CORE_DUMP_"))
      {
        coreDump->print_core_dump();
      }
      else
      {
        RES = 2;
      }
    }
    else if (command.startsWith("DEF_"))
    {
      command = command.substring(4);
      int index = command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).toInt();

      /* */ if (command.startsWith("UICNFG_"))
      {
        SaveStringToFile(String(defaultConfig), ConfigFile);
      }
      else if (command.startsWith("CAR_BAT_")) // car battery
      {
        // later to do
      }
      else if (command.startsWith("SOL_")) // solar pannel
      {
        // later to do
      }
      else if (command.startsWith("CNDTION_")) // conditions
      {
        SaveStringToFile(String(defaultCondition), CondFile);
        jsonCon.readJsonConditionsFromFile(CondFile);
      }
      else if (command.startsWith("VLT_")) // voltage
      {
        EEPROM.writeFloat(E2ADD.VcalCoSave, VcalCoDeflt);
        EEPROM.writeFloat(E2ADD.NegVoltOffsetSave, NegVoltOffsetDeflt);
      }
      else if (command.startsWith("INT_CUR_")) // internal ampermeter
      {
        EEPROM.writeFloat(E2ADD.amp0OffsetSave, amp0OffsetDeflt);
        EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCoDeflt);
      }
      else if (command.startsWith("EXT_CUR_")) // external ampermeter
      {
        EEPROM.writeFloat(E2ADD.ampOffsetSave, ampOffsetDeflt);
        EEPROM.writeFloat(E2ADD.AcalCoSave, AcalCoDeflt);
      }
      else if (command.startsWith("GAS_")) // gas sensor
      {
        EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCoDeflt);
        EEPROM.writeFloat(E2ADD.amp2OffsetSave, amp2OffsetDeflt);
      }
      else if (command.startsWith("OTMP_")) // outside temp pt100
      {
        EEPROM.writeFloat(E2ADD.PT_mvCal_Save, PT_mvCal_Deflt);
      }
      else if (command.startsWith("FLT_1")) // old clean water
      {
        EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMaxDeflt);
        EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMaxDeflt);
      }
      else if (command.startsWith("FLT_2")) // old dirty water
      {
        EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMaxDeflt);
        EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMaxDeflt);
      }
      else if (command.startsWith("FLT_3")) // old gray water
      {
        EEPROM.writeFloat(E2ADD.gryWtrMinSave, gryWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.gryWtrMaxSave, gryWtrMaxDeflt);
        EEPROM.writeFloat(E2ADD.gryWtrMinSave, gryWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.gryWtrMaxSave, gryWtrMaxDeflt);
      }
      else if (command.startsWith("DIM_LIMITS_")) // dimer limits
      {
        for (int i = 0; i < 7; i++) // save dimmer defaults
        {
          EEPROM.writeFloat(E2ADD.dimLimitSave[i], dimLimitDeflt); // limit dimmers to %60
        }
      }
      else if (command.startsWith("ALT_")) // altitude
      {
        EEPROM.writeFloat(E2ADD.pressurCalOffsetSave, pressurCalOffsetDeflt);
      }
      EEPROM.commit();
      loadSavedValue();
      String response = "XrouteAlarm=Default OK\n";
      // myBle.sendString(response.c_str());
      ws.sendToThisClient(response.c_str());
    }
    else if (command.startsWith("CAL_"))
    {
      command = command.substring(4);
      int index = command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).toInt();
      /* */ if (command.startsWith("VLT_"))
      {
        String strVal = command.substring(command.indexOf("=") + 1);
        VcalCo = strVal.toFloat() / volt;
        EEPROM.writeFloat(E2ADD.VcalCoSave, VcalCo);
        EEPROM.commit();
        NegVoltOffset = ADC_LPF(NEG_VOLT_MUX_IN, 5, negv, 0.99);
        EEPROM.writeFloat(E2ADD.NegVoltOffsetSave, NegVoltOffset);
        EEPROM.commit();
        char str[128];
        sprintf(str, "show.txt=\"VcalCo=%f NegVoltOffset=%f\"\n", VcalCo, NegVoltOffset);
        Serial.println(str);
        ws.sendToThisClient(str);
        // myBle.sendString(str);
      }
      else if (command.startsWith("OTMP_")) // outside temp
      {
        // float aimTemp = command.substring(8).toInt() / 10.0;
        // float aimTemp = command.substring(8).toFloat() / 10.0;
        float aimTemp = command.substring(command.indexOf("=") + 1).toFloat() / 10.0;
        float temp = ReadPT100_Temp(pt100mv, 510);
        while (temp < aimTemp && fabs(temp - aimTemp) > 0.1)
        {
          PT_mvCal = ++DFLT_PT_MV_CAL / pt100;
          vTaskDelay(200);
          temp = ReadPT100_Temp(pt100mv, 510);
        }
        vTaskDelay(1000);
        while (temp > aimTemp && fabs(temp - aimTemp) > 0.1)
        {
          PT_mvCal = --DFLT_PT_MV_CAL / pt100;
          vTaskDelay(200);
          temp = ReadPT100_Temp(pt100mv, 510);
        }
        EEPROM.writeFloat(E2ADD.PT_mvCal_Save, PT_mvCal);
        EEPROM.commit();
        char str[128];
        sprintf(str, "show.txt=\"PT_mvCal=%f\"\n", PT_mvCal);
        // myBle.sendString("show.txt=\"PT_mvCal=" + String(PT_mvCal) + "\"\n");
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("INT_CUR_")) // internal current
      {
        A0calCo = static_cast<float>(atoi(command.c_str() + 8)) / (amp0Offset - amp0);
        EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCo);
        EEPROM.commit();
        char str[128];
        sprintf(str, "show.txt=\"A0calCo=%f\"\n", A0calCo);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("ZERO_INT_CUR_")) // internal current offset
      {
        amp0Offset = amp0;
        EEPROM.writeFloat(E2ADD.amp0OffsetSave, amp0Offset);
        EEPROM.commit();
        char str[128];
        sprintf(str, "show.txt=\"amp0Offset=%f\"\n", amp0Offset);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("EXT_CUR_")) // external ampermeter
      {
        float val = command.substring(command.indexOf("=") + 1).toFloat();

        if (ampSenisConnected)
        {
          Serial.println(command.c_str());
          A1calCo = val / (amp1Offset - amp1);
          EEPROM.writeFloat(E2ADD.AcalCoSave, A1calCo);
          EEPROM.commit();
          char str[128];
          sprintf(str, "show.txt=\"A1calCo=%f\"\n", A1calCo);
          // myBle.sendString(str);
          ws.sendToThisClient(str);
        }
        else
        {
          // myBle.sendString("XrouteAlarm=No External Ampermeter Detected !\n");
          ws.sendToThisClient("XrouteAlarm=No External Ampermeter Detected !\n");
          RES = 1;
        }
      }
      else if (command.startsWith("ZERO_EXT_CUR_")) // external current offset
      {
        amp1Offset = amp1;
        EEPROM.writeFloat(E2ADD.ampOffsetSave, amp1Offset);
        EEPROM.commit();
        sprintf(str, "show.txt=\"amp1Offset=%f\"\n", amp1Offset);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("GAS_")) // GAS_1=123
      {
        float val = command.substring(command.indexOf("=") + 1).toFloat();
        A2calCo = (float)val / (amp2Offset - amp2);
        EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCo);
        EEPROM.commit();
        sprintf(str, "show.txt=\"GAS=%f\"\n", A2calCo);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("ZERO_GAS_")) // GAS zero
      {
        amp2Offset = amp2;
        EEPROM.writeFloat(E2ADD.amp2OffsetSave, amp2Offset);
        EEPROM.commit();
        sprintf(str, "show.txt=\"Gas Offset=%f\"\n", amp2Offset);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("MAX_DIM_")) // maximum Dimmer MAX_DIM_1=123
      {
        float val = command.substring(command.indexOf("=") + 1).toFloat();
        unsigned int dimNum = command.substring(command.lastIndexOf("_") + 1).toInt() - 1;
        dimLimit[dimNum] = val / 255;
        dimTmp[dimNum] = (double)32767 * dimLimit[dimNum];
        // save
        EEPROM.writeFloat(E2ADD.dimLimitSave[dimNum], dimLimit[dimNum]);
        EEPROM.commit();
        // add debug print
        // Serial.printf("DIM%d val%f", dimNum, val);
      }
      else if (command.startsWith("BATT_CAP_")) // Battery Cap BAT_CAP=345
      {
        int val = command.substring(command.indexOf("=") + 1).toInt();
        DFLT_BATT_CAP = val;
        EEPROM.writeFloat(E2ADD.batteryCapSave, DFLT_BATT_CAP);
        EEPROM.commit();
        batteryCap = DFLT_BATT_CAP;
        sprintf(str, "BattCapTxt.val=%d\n", (int)batteryCap);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("MAX_FLT_")) // MAX_FLT_1
      {
        if (index == 1)
        {
          clnWtrMax = clnWtr;
          EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMax);
          EEPROM.commit();
          sprintf(str, "show.txt=\"FLT_1_Max=%f\"\n", index, clnWtrMax);
        }
        if (index == 2)
        {
          drtWtrMax = drtWtr;
          EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMax);
          EEPROM.commit();
          sprintf(str, "show.txt=\"FLT_2_Max=%f\"\n", index, drtWtrMax);
        }

        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("MIN_FLT_")) // MIN_FLT_1
      {
        if (index == 1)
        {
          clnWtrMin = clnWtr;
          EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMin);
          EEPROM.commit();
          sprintf(str, "show.txt=\"FLT_1_Min=%f\"\n", clnWtrMin);
        }
        if (index == 2)
        {
          drtWtrMin = drtWtr;
          EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMin);
          EEPROM.commit();
          sprintf(str, "show.txt=\"FLT_2_Min=%f\"\n", drtWtrMin);
        }
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("DIM_MAXES_"))
      {
        EEPROM.writeFloat(E2ADD.dimLimitSave[index - 1], dimLimit[index - 1]);
        EEPROM.commit();
        sprintf(str, "XrouteAlarm=Dimer%d Limit Saved OK! \n", index);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("GYRO_ORI_")) // gyro oriantation GYRO_ORI_1=XY01
      {
        String val = command.substring(command.indexOf("=") + 1);
        GyroOriantation.setCharAt(0, val[0]);
        GyroOriantation.setCharAt(1, val[1]);
        GyroOriantation.setCharAt(2, val[2]);
        GyroOriantation.setCharAt(3, val[3]);
        GyroOriantation.setCharAt(4, '\0');
        // Serial.println("BEFOR SAVE ORIANTATION:" + GyroOriantation);
        EEPROM.writeString(E2ADD.GyroOriantationSave, GyroOriantation);
        EEPROM.commit();
        char strTmp[32];
        EEPROM.readString(E2ADD.GyroOriantationSave, strTmp, 16);
        GyroOriantation = String(strTmp);
        sprintf(str, "XrouteAlarm=Gyro Saved Ok! \n");
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else if (command.startsWith("ALT_")) // ALTITUTE ALT_1=123
      {
        float aimAlt = command.substring(command.indexOf("=") + 1).toFloat();
        float tempAlt = psiToMeters(BARO.readPressure(PSI) - pressurCalOffset);
        float error = 1; // to get into the loop
        float pressurCalOffsetTemp = pressurCalOffset;
        sprintf(str, "show.txt=\"Calibrating=\"\n");
        // myBle.sendString(str);
        ws.sendToThisClient(str);

        while (fabs(error) > 0.1)
        {
          if (isnan(pressurCalOffsetTemp)) // for somthimes crashes
          {
            pressurCalOffsetTemp = 0;
            Serial.println("ERROR");
          }
          error = aimAlt - tempAlt;
          pressurCalOffsetTemp += (error / 5000);
          tempAlt = psiToMeters(BARO.readPressure(PSI) - pressurCalOffsetTemp);
          Serial.println("PrOffset=" + String(pressurCalOffsetTemp, 5));
          sprintf(str, "M.Pre.val=%d\n", (int)tempAlt);
          // myBle.sendString(str);
          ws.sendToThisClient(str);
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (!isnan(pressurCalOffsetTemp)) // for somthimes crashes
        {
          pressurCalOffset = pressurCalOffsetTemp;
          EEPROM.writeFloat(E2ADD.pressurCalOffsetSave, pressurCalOffset);
          EEPROM.commit();
        }
        sprintf(str, "show.txt=\"pressurCalOffset=%f\"n", pressurCalOffset);
        // myBle.sendString(str);
        ws.sendToThisClient(str);
      }
      else
      {
        RES = 4;
      }
    }
    else if (command.startsWith("SAVE_"))
    {
      command = command.substring(5);
      int index = command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).toInt();
      /**/ if (command.startsWith("GYRO_ORI_"))
      {
        int index = command.substring(command.lastIndexOf("_") + 1, command.indexOf("=")).toInt();

        GyroOriantation = String(command.substring(16, 5).c_str());
        EEPROM.writeString(E2ADD.GyroOriantationSave, GyroOriantation);
        EEPROM.commit();
        String strTmp = EEPROM.readString(E2ADD.GyroOriantationSave);
        GyroOriantation = strTmp;
      }
      else if (command.startsWith("WIFI_SSID_")) // WIFI_SSID_1=ssid
      {
        String ssid = command.substring(command.indexOf("=") + 1);
        wifi_WebSocket_Settings.set<String>(NetworkKeys::WifiSSID_NEW, ssid);
        wifi_WebSocket_Settings.set<bool>(NetworkKeys::NewConfigIsAvailble, true);
        Serial.println("Wifi SSID:" + ssid);
      }
      else if (command.startsWith("WIFI_PASS_")) // WIFI_PASS_1=pass\r
      {
        String pass = command.substring(command.indexOf("=") + 1);
        wifi_WebSocket_Settings.set<String>(NetworkKeys::WifiPassword_NEW, pass);
        wifi_WebSocket_Settings.set<bool>(NetworkKeys::NewConfigIsAvailble, true);
        String str = "WIFI_PASS_CHANGED=OK\n";
        ws.sendToAll(str.c_str());
      }
      else if (command.startsWith("WIFI_MODE_")) // WIFI_MODE_1=STA /
      {
        String mode = command.substring(command.indexOf("=") + 1);
        if (mode == "STA")
        {
          wifi_WebSocket_Settings.set<wifi_mode_t>(NetworkKeys::STA_AP, WIFI_MODE_STA);
        }
        else if (mode == "AP")
        {
          wifi_WebSocket_Settings.set<wifi_mode_t>(NetworkKeys::STA_AP, WIFI_MODE_AP);
        }
        else if (mode == "STA_AP")
        {
          wifi_WebSocket_Settings.set<wifi_mode_t>(NetworkKeys::STA_AP, WIFI_MODE_APSTA);
        }
        // ws.switchMode(wifi_WebSocket_Settings.get<wifi_mode_t>(NetworkKeys::STA_AP));TODO : switch mode causes crash so lets restart for now
        String str = "WIFI_MODE_CHANGED_OK\n";
        ws.sendToAll(str.c_str());
      }
      else if (command.startsWith("HOST_NAME_"))
      {
        String hostName = command.substring(command.indexOf("=") + 1);
        wifi_WebSocket_Settings.set<String>(NetworkKeys::HostName, hostName);
        String str = "HOST_NAME_CHANGED_OK\n";
        ws.sendToAll(str.c_str());
      }
      else if (command.startsWith("WIFI_AP_PASS_"))
      {
        String pass = command.substring(command.indexOf("=") + 1);
        wifi_WebSocket_Settings.set<String>(NetworkKeys::ApPassword, pass);
        String str = "WIFI_AP_PASS_CHANGED_OK\n";
        ws.sendToAll(str.c_str());
      }
      else
      {
        RES = 5;
      }
    }
    else
    {
      RES = 10;
    }
    if (RES)
    {
      Serial.println("[PARSING ERROR] : " + command + " : " + String(RES));
      // empty buffer
      accumulatedData.clear();
      command.clear();
      RES = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    MeasurmentTaskPause = false;
  }
  // if there is missing '\n' print error
  if (accumulatedData.length() > 0)
  {
    String errorMessage = "Missing[\\n]:" + String(accumulatedData.c_str());
    Serial.println(errorMessage);
    ws.sendToThisClient(errorMessage.c_str());
    accumulatedData.clear();
  }
}
std::deque<String> cmdQueue;
SemaphoreHandle_t cmdQueueMutex;
void onDataReceived(uint8_t *pData, size_t length)
{
  // You no longer need any security checks here.
  // This function is now ONLY called for data from already-authenticated, whitelisted devices.
  // Directly process the command from the trusted device.
  // processReceivedCommandData(pCharacteristic, pData, length);
  String msg = String(reinterpret_cast<char *>(pData), length);
  if (cmdQueue.size() < 8) // Optional limit to prevent unbounded growth
  {
    cmdQueue.push_back(String(msg));
  }
  else
  {
    Serial.println("🔷⚠️ Command queue full, dropping incoming message");
  }
}
void sendCmdToExecute(char *str)
{
  // Simulate BLE data reception
  uint8_t *pData = reinterpret_cast<uint8_t *>(str); // Removed const_cast - ensure str is mutable if needed, or cast differently
  size_t length = strlen(str);
  processReceivedCommandData(pData, length);
}
void cmdQueTask(void *pvParameters)
{
  while (true)
  {
    String cmd;
    while (!cmdQueue.empty())
    {
      cmd = cmdQueue.front();
      cmdQueue.pop_front();
      sendCmdToExecute((char *)cmd.c_str());
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void serialCB()
{
  static String serialBuffer;
  while (Serial.available())
  {
    char c = Serial.read();

    // Handle backspace or delete characters
    if ((c == '\b' || c == 127) && serialBuffer.length() > 0)
    {
      serialBuffer.remove(serialBuffer.length() - 1); // Remove last char from buffer
      Serial.print("\b \b");                          // Erase character on the serial monitor
    }
    // Handle newline character (command submission)
    else if (c == '\n' || c == '\r')
    {
      Serial.println(); // Move to the next line in the monitor
      if (serialBuffer.length() > 0)
      {
        serialBuffer += '\n';
        cmdQueue.push_back(String(serialBuffer));
        // sendCmdToExecute((char *)serialBuffer.c_str());// it will drow all the resources from this event that should be normally an small code , its an event
        serialBuffer = ""; // Clear the buffer for the next command
      }
    }
    // Handle regular, printable characters
    else if (isPrintable(c))
    {
      serialBuffer += c; // Add character to the buffer
      Serial.print(c);   // Echo character to the serial monitor
    }
  }
}
//--------------------
void setup()
{
  // low levels and hardware
  if (true)
  {
    pinMode(34, INPUT_PULLUP); // Dimmer Protection PIN 34
    attachInterrupt(digitalPinToInterrupt(34), dimmerShortCircuitIntrupt, FALLING);
    Serial.begin(115200);
    coreDump = new MyCoreDump();
    Serial.println("\n//======STARTING=====//");
    // atach an intrupt for incomming serial data
    Serial.onReceive(serialCB);
    initRelay();
    initLED_PWM();
    initADC();
    strip.begin();
    uint32_t flashSize = ESP.getFlashChipSize();
    float flashSizeMB = (float)flashSize / (1024.0 * 1024.0); // MB
    Serial.print("FlashSize:");
    Serial.println(flashSizeMB);
    Serial.println("Version:" + Version);
    EEPROM.begin(512);
    loadSavedValue();
  }

  // SPIFF
  if (SPIFFS.begin(true))
  {
    Serial.println("SPIFF OK !");
    loadStateFromFile();
    // UI Config File
    if (!SPIFFS.exists(ConfigFile))
    {
      if (!SaveStringToFile(String(defaultConfig), ConfigFile))
      {
        Serial.println("Error Saving: " + String(ConfigFile));
      }
    }
    String fileContent = readStringFromFile(ConfigFile);
    // Serial.println("Config File Content: " + fileContent);
    // CONDITIONS
    // SaveStringToFile(String(defaultCondition), CondFile); // for test and you need to disable it otherwise it will default it again every reset
    if (!SPIFFS.exists(CondFile))
    {
      if (!SaveStringToFile(String(defaultCondition), CondFile))
      {
        Serial.println("Error Saving: " + String(CondFile));
      }
    }
    fileContent = readStringFromFile(CondFile);
    // Serial.println("Condition File Content: " + fileContent);
  }
  else
  {
    Serial.println("SPIFF ERROR !");
  }

  // licensing
  if (true)
  {
    xrtLcns = new RegDev(RegFilePath);
    xrtLizing = new Leasing(UpTimeFilePath);
  }

  // Config the Conditions
  if (true)
  {
    conditionSetVariables(&v, &a0, &a1, &w, &b, &cwPrcnt, &dwPrcnt, &gwPrcnt,
                          &digitalTemp, &digitalHum, &digitalAlt, &pt100, &a2,
                          &battHourLeft, &motorWay);
    setCmdFunction(&sendCmdToExecute);
    getRelayStateFunction(&relState_0_15);
    getDimValueFunction(&getDimVal);
    setCondCreatorFunction(&createCondition);
    setTimerCondCreatorFunction(&createTimerCondition);
    jsonCon.readJsonConditionsFromFile(CondFile);
  }

  // lisences
  if (true)
  {
    GyroLicense = new lisence("Gyro", "G9933");   // Key for Gyro
    VoiceLicense = new lisence("Voice", "V5612"); // Key For Voice
    // Serial.println("General Lisence:" + GeneralLisence);
    // giveMeMacAdress();
  }

  // user infos
  if (true)
  {
    // wifi & WS
    if (!wifi_WebSocket_Settings.begin())
    {
      Serial.println("[" + wifi_WebSocket_Settings.getPath() + "]No valid JSON found, starting fresh.");
    }
    else
    {
      Serial.println("[" + wifi_WebSocket_Settings.getPath() + "]==> LOADED OK !");
    }
    // other settings
    if (!other_Settings.begin())
    {
      Serial.println("[" + other_Settings.getPath() + "]No valid JSON found, starting fresh.");
    }
    else
    {
      Serial.println("[" + other_Settings.getPath() + "]==> LOADED OK !");
      uiConfigIndex = other_Settings.get<uint32_t>(otherKeys::UI_CONFIG_INDEX, 1);
      other_Settings.saveIfChanged();
    }
  }

  // Network AND websocket
  if (true)
  {
    // load new Wifi config
    bool newConfigIsAvailble = wifi_WebSocket_Settings.get<bool>(NetworkKeys::NewConfigIsAvailble, false);
    String newSsid = wifi_WebSocket_Settings.get<String>(NetworkKeys::WifiSSID_NEW, "");
    String newPass = wifi_WebSocket_Settings.get<String>(NetworkKeys::WifiPassword_NEW, "");
    wifi_WebSocket_Settings.get<int>(NetworkKeys::AtempResault, -1); //-1 is default
    wifi_WebSocket_Settings.saveIfChanged();                         // for absulute first run
    if (newConfigIsAvailble & newSsid.length() > 0 & newPass.length() > 0)
    {
      // try to connect to it if its ok to connect to it
      int res = testWifi(newSsid, newPass);
      wifi_WebSocket_Settings.set<int>(NetworkKeys::AtempResault, res);
      wifi_WebSocket_Settings.set<bool>(NetworkKeys::NewConfigIsAvailble, false);
      if (res == WL_CONNECTED)
      {
        wifi_WebSocket_Settings.set<String>(NetworkKeys::WifiSSID, newSsid);
        wifi_WebSocket_Settings.set<String>(NetworkKeys::WifiPassword, newPass);
      }
    }
    // Configure network
    String ssid = wifi_WebSocket_Settings.get<String>(NetworkKeys::WifiSSID, "LabobinX_2.4G");
    String pass = wifi_WebSocket_Settings.get<String>(NetworkKeys::WifiPassword, "102030405060");
    String defApName = "Xroute-AP" + getDeviceCode();
    String apName = wifi_WebSocket_Settings.get<String>(NetworkKeys::ApName, defApName);
    String apPass = wifi_WebSocket_Settings.get<String>(NetworkKeys::ApPassword, "12345678");
    String hostName = wifi_WebSocket_Settings.get<String>(NetworkKeys::HostName, "xroute");
    wifi_mode_t mode = wifi_WebSocket_Settings.get<wifi_mode_t>(NetworkKeys::STA_AP, WIFI_MODE_STA);
    int port = wifi_WebSocket_Settings.get<int>(NetworkKeys::Port, 81);
    wifi_WebSocket_Settings.saveIfChanged();
    ws.setAP(apName.c_str(), apPass.c_str());
    ws.setSTA(ssid.c_str(), pass.c_str());
    ws.setHostname(hostName.c_str());
    ws.setPort(port);
    ws.onJson([](JsonDocument &doc)
              {
                // 1) Treat the entire doc as a JsonObject
                JsonObject root = doc.as<JsonObject>();
                // 2) Iterate over its key/value pairs. If you know there’s exactly one wrapper,
                //    you can just grab the first pair.
                if (root.size() == 0)
                {
                  Serial.println("No keys found in incoming JSON");
                  return;
                }
                // Take the first key/value pair in `root`
                JsonPair kv = *root.begin();
                const char *wrapperKey = kv.key().c_str();
                JsonVariant wrappedValue = kv.value();
                Serial.print("Detected wrapper key: ");
                Serial.println(wrapperKey);
                // Now `wrappedValue` is whatever was under that key. If you know it’s an array:
                if (!wrappedValue.is<JsonArray>())
                {
                  Serial.println("Warning: expected an array under the wrapper key");
                  return;
                }
                JsonArray arr = wrappedValue.as<JsonArray>();
                Serial.print("Array size under \"");
                Serial.print(wrapperKey);
                Serial.print("\": ");
                Serial.println(arr.size());
                // Example: if you want to print the first element of that array:
                if (arr.size() > 0 && arr[0].is<JsonObject>())
                {
                  Serial.println("First element:");
                  String data;
                  // serializeJsonPretty(arr[0].as<JsonObject>(), data);// it cuases ~2.2X more size
                  serializeJson(arr[0].as<JsonObject>(), data);

                  // Serial.println("==========="+data+"===========");
                  if (strncmp(wrapperKey, jsonKeys::UI_CONFIG, 9) == 0)
                  {
                    bool status = SaveStringToFile(data, ConfigFile);
                    if (status == true)
                    {
                      ws.sendToAll("UiSevadSuccessful\n");
                      uiConfigIndex++;
                      other_Settings.set<uint32_t>(otherKeys::UI_CONFIG_INDEX, uiConfigIndex);
                      // ws.SendToAllExcludeClient("NOTIF_NEW_UI_CONFIG_1\n", ws.getClient());
                    }
                    else
                    {
                      ws.sendToAll("UiSevadError\n");
                    }
                  }
                  else if (strncmp(wrapperKey, jsonKeys::CONDITON_CONFIG, 11) == 0)
                  {
                    jsonCon.saveConditionsFileFromString(CondFile, data);
                    Serial.println("Condition Received Successfully");
                    Serial.println("Checking file....");
                    if (jsonCon.isJsonFileOk(CondFile))
                    {
                      // myBle.sendString("ConditionSevadSuccessful\n");
                      ws.sendToAll("ConditionSevadSuccessful\n");
                      Serial.println("Condition File is ok");
                      Serial.println("deleting old conditions vector");
                      cndtions.clear(); // it will automatically call all the destroyers
                      // there was a problem with destructor and vector that it executes destructor unwanted time and i got negative numbers so i comment destructer --
                      Conditions::ConditionCount = 0;
                      Serial.println("jsonCon.readJsonConditionsFromFile(CondFile)");
                      jsonCon.readJsonConditionsFromFile(CondFile);
                    }
                    else
                    {
                      // myBle.sendString("ConditionSevadError\n");
                      ws.sendToAll("ConditionSevadError\n");
                    }
                  }
                  else
                  {
                    // message not valid key
                    Serial.println("message not valid key");
                  }
                }
                //
              });
    cmdQueueMutex = xSemaphoreCreateMutex();
    xTaskCreate(cmdQueTask, "CmdQueTask", 4 * 1024, NULL, 1, NULL);
    ws.setMaxCmdPkgSize(128);
    ws.onCommand([](const char *msg)
                 {
                   // uint32_t time=micros();
                   if (cmdQueue.size() < 16) // Optional limit to prevent unbounded growth
                   {
                     cmdQueue.push_back(String(msg));
                   }
                   else
                   {
                     Serial.println("🌐⚠️  Command queue full, dropping incoming message");
                   }
                   // Serial.println("time="+String(micros()-time));
                 });

    ws.onUpdate([](const char *msg, size_t length)
                {
                  Serial.println(".");
                  while (UDP_bufferBussy)
                  {
                    vTaskDelay(pdMS_TO_TICKS(1));
                  }
                  UDP_bufferBussy = true;
                  UDP_buffLen = length;
                  memcpy(UDP_buffer, msg, length);
                  UDP_dataReady = true;
                  //
                });
    ws.begin(mode);
  }

  // Tasks
  if (true)
  {
    xTaskCreatePinnedToCore(
        MeasurmentTask,
        "MeasurmentTask",
        2300, // ✔️
        NULL,
        1,
        &MeasurmentTaskHandle,
        1);
    xTaskCreatePinnedToCore(
        DimerTask,
        "DimerTask",
        2.5 * 1024, // ✔️
        NULL,
        2,
        &DimerTask_Handle,
        1);
    xTaskCreatePinnedToCore(
        adcReadingTask,
        "ADC READING TASK",
        1.5 * 1024, // ✔️
        NULL,
        1,
        &adcReadingTask_Handle,
        1);
    xTaskCreatePinnedToCore(
        led_indicator_task,
        "led_indicator_task",
        1.2 * 1024, // ✔️
        NULL,
        1,
        &led_indicator_task_Handle,
        0);
    xTaskCreatePinnedToCore(
        OVR_CRNT_PRTCT_TASK,
        "OVR_CRNT_PRTCT_TASK",
        1.5 * 1024, // ✔️
        NULL,
        1,
        &OVR_CRNT_PRTCT_TASK_Handle,
        0);
    xTaskCreatePinnedToCore(
        I2C_SENSORS_TASK, //-------------STACK optimized up to here
        "I2C_SENSORS_TASK",
        3 * 1024, // ❌
        NULL,
        1,
        &I2C_SENSORS_TASK_Handle,
        1);
    xTaskCreatePinnedToCore(
        BatteryTask,
        "BatteryTask",
        1.5 * 1024, // ✔️
        NULL,
        1,
        &BatteryTask_Handle,
        1);
    xTaskCreatePinnedToCore(
        ConditionsTask,
        "ConditionsTask",
        5 * 1024, // ❌
        NULL,
        1,
        &ConditionsTask_Handle,
        1);
    // xTaskCreatePinnedToCore(
    //     Reg_Uptime_Task,
    //     "regControlTask",
    //     4 * 1024, // ✔️
    //     NULL,
    //     1,
    //     &Reg_Uptime_Task_Handle,
    //     1);
    // CPU
    // xTaskCreatePinnedToCore(
    //     cpuMonitoringTask,
    //     "cpuMonitoringTask",
    //     3 * 1024,
    //     NULL,
    //     1,
    //     NULL,
    // 0);
    // RAM
    xTaskCreatePinnedToCore(
        ramMonitorTask,
        "ramMonitorTask",
        1024, // stack size
        NULL,
        1,
        NULL, 0);
  }
}
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(100));
}
void loadSavedValue()
{
  if (EEPROM.readUInt(E2ADD.E2promFirsTime) == E2PROM_NOT_FIRST_TIME_RUN_VAL)
  {
  }
  else
  {
    defaultCalibrations();
  }
  blePass = EEPROM.readUInt(E2ADD.blePassSave);
  pressurCalOffset = EEPROM.readFloat(E2ADD.pressurCalOffsetSave);
  accXValueOffset = EEPROM.readFloat(E2ADD.accXValueOffsetSave);
  accYValueOffset = EEPROM.readFloat(E2ADD.accYValueOffsetSave);
  NegVoltOffset = EEPROM.readFloat(E2ADD.NegVoltOffsetSave);
  VcalCo = EEPROM.readFloat(E2ADD.VcalCoSave);
  amp1Offset = EEPROM.readFloat(E2ADD.ampOffsetSave);
  A1calCo = EEPROM.readFloat(E2ADD.AcalCoSave);
  amp0Offset = EEPROM.readFloat(E2ADD.amp0OffsetSave);
  A0calCo = EEPROM.readFloat(E2ADD.A0calCoSave);
  amp2Offset = EEPROM.readFloat(E2ADD.amp2OffsetSave);
  A2calCo = EEPROM.readFloat(E2ADD.A2calCoSave);
  clnWtrMin = EEPROM.readFloat(E2ADD.clnWtrMinSave);
  clnWtrMax = EEPROM.readFloat(E2ADD.clnWtrMaxSave);
  drtWtrMin = EEPROM.readFloat(E2ADD.drtWtrMinSave);
  drtWtrMax = EEPROM.readFloat(E2ADD.drtWtrMaxSave);
  gryWtrMin = EEPROM.readFloat(E2ADD.gryWtrMinSave);
  gryWtrMax = EEPROM.readFloat(E2ADD.gryWtrMaxSave);
  batteryCap = EEPROM.readFloat(E2ADD.batteryCapSave);
  DFLT_BATT_CAP = batteryCap;
  battFullVoltage = EEPROM.readFloat(E2ADD.battFullVoltageSave);
  DFLT_BATT_FULL_VOLT = battFullVoltage;
  battEmptyVoltage = EEPROM.readFloat(E2ADD.battEmptyVoltageSave);
  DFLT_BATT_EMPTY_VOLT = battEmptyVoltage;
  cableResistance = EEPROM.readFloat(E2ADD.cableResistanceSave);
  PT_mvCal = EEPROM.readFloat(E2ADD.PT_mvCal_Save);

  for (int i = 0; i < 7; i++)
  {
    dimLimit[i] = EEPROM.readFloat(E2ADD.dimLimitSave[i]);
  }
  char strTmp[64];
  // GeneralLisence = EEPROM.readString(E2ADD.licenseStatSave);
  EEPROM.readString(E2ADD.licenseStatSave, strTmp, 32);
  GeneralLisence = String(strTmp);
  if (strncmp(strTmp, "Gyro", 4) != 0) // toole takhminiye maximum baraye lisense alan na badan voice + gyro
  {
    EEPROM.writeString(E2ADD.licenseStatSave, "Gyro:False,Voice:False");
    EEPROM.commit();
    EEPROM.readString(E2ADD.licenseStatSave, strTmp, 32);
    GeneralLisence = String(strTmp);
  }
  // String tmp = EEPROM.readString(E2ADD.GyroOriantationSave);
  EEPROM.readString(E2ADD.GyroOriantationSave, strTmp, 16);
  GyroOriantation = String(strTmp);
  // Serial.println("LOADGYRO ORIANTATION:" + GyroOriantation);
  if (strncmp(strTmp, "XY", 2) != 0 && strncmp(strTmp, "XZ", 2) != 0 &&
      strncmp(strTmp, "YX", 2) != 0 && strncmp(strTmp, "YZ", 2) != 0 &&
      strncmp(strTmp, "ZX", 2) != 0 && strncmp(strTmp, "ZY", 2) != 0)
  {
    // Serial.println("GyroReset---------");
    EEPROM.writeString(E2ADD.GyroOriantationSave, "XY00"); // DefaulOriantaton
    EEPROM.commit();
    EEPROM.readString(E2ADD.GyroOriantationSave, strTmp, 16);
    GyroOriantation = String(strTmp);
  }
}
void ws2812Blink(int color, int times, int interval_ms, float intensity)
{
#define MIN_LIGHT 1
  int MAX_LIGHT = 255 * intensity;
  int STEP_INTERVAL_MS = interval_ms;
  for (int i = 0; i < times; i++)
  {
    for (int i = MIN_LIGHT; i < MAX_LIGHT; i += 2)
    {
      strip.setLedColorData(0, color);
      strip.setBrightness(i);
      strip.show();
      vTaskDelay(pdMS_TO_TICKS(STEP_INTERVAL_MS));
    }
    for (int i = MAX_LIGHT; i >= MIN_LIGHT; i -= 2)
    {
      strip.setLedColorData(0, color);
      strip.setBrightness(i);
      strip.show();
      vTaskDelay(pdMS_TO_TICKS(STEP_INTERVAL_MS));
    }
  }
}
void initMPU()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}
void dimmerShortCircuitIntrupt()
{
  static unsigned long dt = 0, t = 0, lastT = 0, freq = 0;
  static int cntr = 0;
  static unsigned long sum = 0;
  static int dimNum;

  t = micros();
  dt = t - lastT;
  lastT = t;
  freq = 1000000 / dt;

  if (freq > 900 && freq < 1700)
  {
    cntr++;
    sum += freq;
  }
  else
  {
    cntr = 0;
    sum = 0;
    return;
  }
  if (cntr > 50)
  {
    dimShortNum = round(((sum / cntr) - 950) / 100);
    dimShortFlg = true;
    cntr = 0;
    sum = 0;
  }
}
/*  new problems
// ino haminjoori minevisam ke commit konam bebinam ye shakhe jadid dar miyad ya na
4- neveshtane scadual condition
5- timeout vase update
6- dimere rate ziyad drop beshe
7- sharte larzeshe gyro baraye jologiri az dozdi
8- ba release dimmer ali behem dastore saveStatesToFile(); ro bedahad / ali mige rooye on crash age betooni bezari aliye / behatresh ine ke ba timer befahmi ke dimer dige change nemishe va oon moghe savestate koni
9-
*/
// sendCmdToExecute needs wait for already incomming tasks
//+++++++++++++++++++++++TO DO
//   ba ble nemishe JSONE hajme balaye 256B ro ersal kard masalan age GET_DEV_INFO_JSON_1 bezani nesfe miyad
//   dakhele loope Vcal to ya dakhele loope Tcal to infinit loop nabayad beshe
//   voltage ke yehoyi biyad payin ya inke voltage eshtebah kalibre beshe rele vel mikone
//   amper ke eshtebah kalibre beshe eshtebahi mire too ye protection
//   *vaghti raft tooye protection mode bayad message bede ke amper kheyli ziyade va badesh ke ok shod bayad ba ye payame ok az khata darbiyad
//   *password bezar baraye blt
//   ezafe kardane ye delaye koochik vaghti ke calibratione chizi ro ancam dadim ta inke baes beshe tooye textbox dide beshe ta beshe baraye meghdare defaultha unu khoondesh ya mishe printesh kard too serial
//   avaz kardane hajme flash baraye inke alan flashet 2 barabar shode
//   Saman for ALI: ye geraphice khoob baraye ali baraye gyro hazer kon
//   connect ya disconnect shodane hame chizo mitooni tashkhis bedi az tarighe sathe voltaga
//   VAGHTI GYRO RESET MISHE LOW PASS SEFR BESHE CHONKE KHEYLI TOOL MIKESHE
//   baraye 24 volt bayad devidere voltago dorost fekr koni barash chon alan majboori avazesh koni
//   az tabeye constrain baraye limit kardane valuehat estefade kon tooye hamejaye kod ke value ha alaki naran asemoono....
//   detect floaters
//   problems
//    i dont send the value from APDIM to other BLE devices in line 920