//====================================================libraries
#define __________________________________________INCLUDES
#ifdef __________________________________________INCLUDES
#include <sdkconfig.h>
#include <esp_bt_device.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "defaultValues.h"
#include "save_load.h"
#include <BluetoothSerial.h>
#include <BLESerial.h>
#include <BLEDevice.h>
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
#endif
#define __________________________________________VAR_DEF
#ifdef __________________________________________VAR_DEF
//*******************VERSION CONTROLS
/* 1.0.3
 1-external ampermeter activated
 2-default calibration for each value added
 3-pt100 easy calibration
 4-loading DFLT values after defaultCalib.. function
 5-DIFFERENT FREQUANCY PWM
 6-DIMMER short circuit detection
 7-low power mode active
 8-low power mode in battery percent
 9-battery type selection by Full voltage
 10-gyro zeroing fast
*/
// 2.0.7/4.0.7 increasing tasks ram by 1KB to prevent crashing : not tested
// String Version = "4.0.7"; 24V version
// 2.0.8 adding save to file for state recovery after crashes
String Version = "2.1.0";
//========Update
#include "Update.h"
#include "AESLib.h"
long int updateLen;
#define CHUNK_SIZE 512
// #define CHUNK_SIZE 256
uint8_t dataBuff[CHUNK_SIZE];
AESLib aes;
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
float accXValue = 0, accYValue = 0, accZValue = 0, len = 0, alpha = 0, accSensitivity = 0;
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
void ws2812Blink(int color);
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
// END---------------------------------HUMIDITY SENSORS
//------------------------------------VOLTAGES
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
BluetoothSerial SerialBT;
int blePass;
//--------------------------DATA'S
extern struct bleData BLE_DATA;
char mainRxStr[128];
char mainTxStr[128];
bool mainStrIsFree = true;
// END-----------------------DATA's
#endif
#define DEBUG
//===========================Files
const String statesFile = "/LastStates.txt";
#include <ButtonConfig.h>
const String ConfigFile = "/ConfigFile.txt";
#include <defaultConditions.h>
extern const char *defaultCondition;
String confAndCondStrBuffer = "";
const String CondFile = "/CondFile.txt";
//===========================Conditions
#include <vector>
#include <conditions.h>
std::vector<Conditions> cndtions;
#include <jsonCondition.h>
ConditionsReader jsonCon;
//===========================CLASSES
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
//----------------------Barometer
TaskHandle_t ramMonitorTaskHandle;
//----------------------Low Voltage And Critical Voltage
float BatteryLowPrcnt = 11.0;
uint16_t BatteryLowPrcntRelays = 0b000000000000000;
uint8_t BattLowPrcntDimers = 0b0000000;
float BattCriticalPrcnt = 10.0;
uint16_t BattCriticalPrcntRelays = 0b0000000000000000;
uint8_t BattCriticalPrcntDimers = 0b0000000;
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
      if (mainGyro == ADD_INTERNAL_GYRO && GyroLicense->isActive() == true)
      {
        float tempx, tempy;
        readaxels(ADD_INTERNAL_GYRO, &tempx, &tempy);
        accXValue = LOW_PASS_FILTER(tempx, accXValue, 0.985);
        accYValue = LOW_PASS_FILTER(tempy, accYValue, 0.985);

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
void sendAllcalibrations();
void sendCmndToMainStringProcessorTask(char *str);
void dimmerShortCircuitIntrupt();
void defaultCalibrations();
int dimShortFlg = false;
int dimShortNum = 0;
//-------------------------------------------------TASKS
void ConditionsTask(void *parameters)
{
  Serial.println("Conditions Task Started");
  for (;;)
  {
    for (int i = 0; i < Conditions::getCount(); i++)
    {
      cndtions[i].doWork();
    }
    vTaskDelay(500);
  }
}
void loadStateFromFile()
{
  Serial.println("Loading Last States");
  // Open the file for reading
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

  // Optionally print the loaded values for debugging
  Serial.printf("Loaded state: RELAYS:%d,D0:%f,D1:%f,D2:%f,D3:%f,D4:%f,D5:%f,D6:%f\n",
                RELAYS.relPos, dimTmp[0], dimTmp[1], dimTmp[2], dimTmp[3], dimTmp[4], dimTmp[5], dimTmp[6]);
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
  // SetNextion(RELAYS.relPos);
  // volt = ADC_LPF(VOLT_MUX_IN, 5, volt, 0.0);
  // amp0 = ADC_LPF(AMPER0_MUX_IN, 5, amp0, 0.0);
  // amp1 = ADC_LPF(AMPER_MUX_IN, 5, amp1, 0.0);
  // amp2 = ADC_LPF(AIR_QUALITY_MUX_IN, 5, amp2, 0.0);
  // clnWtr = ADC_LPF(GAUGE1_MUX_IN, 5, clnWtr, 0.0);
  // drtWtr = ADC_LPF(GAUGE2_MUX_IN, 5, drtWtr, 0.0);
  // gryWtr = ADC_LPF(GAUGE3_MUX_IN, 5, gryWtr, 0.0);
  // pt100 = ADC_LPF(PT100_MUX_IN, 5, pt100, 0.0);
  vTaskDelay(1000); // wait for adc reading task to loop for a while
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
    if (cwPrcnt > 1820)
      cwPrcnt = 1820;
    if (dwPrcnt > 1820)
      dwPrcnt = 1820;
    if (gwPrcnt > 1820)
      gwPrcnt = 1820;
    if (cwPrcnt < 0)
      cwPrcnt = 0;
    if (dwPrcnt < 0)
      dwPrcnt = 0;
    if (gwPrcnt < 0)
      gwPrcnt = 0;
    pt100mv = pt100 * PT_mvCal;
    sprintf(str, "M.V1.val=%d\xFF\xFF\xFF", (int)v);
    SendToAll(str);
    sprintf(str, "M.A0.val=%d\xFF\xFF\xFF", (int)a0);
    SendToAll(str);
    sprintf(str, "M.A1.val=%d\xFF\xFF\xFF", (int)a1);
    SendToAll(str);
    sprintf(str, "M.W1.val=%d\xFF\xFF\xFF", (int)w);
    SendToAll(str);
    sprintf(str, "M.BatPr.val=%d\xFF\xFF\xFF", (int)b);
    SendToAll(str);
    sprintf(str, "M.G1TXT.val=%d\xFF\xFF\xFF", ((int)cwPrcnt) / 10 * 10);
    SendToAll(str);
    sprintf(str, "M.G2TXT.val=%d\xFF\xFF\xFF", ((int)dwPrcnt) / 10 * 10);
    SendToAll(str);
    sprintf(str, "M.G3TXT.val=%d\xFF\xFF\xFF", ((int)gwPrcnt) / 10 * 10);
    SendToAll(str);
    sprintf(str, "M.T1.val=%d\xFF\xFF\xFF", (int)(10 * ReadPT100_Temp(pt100mv, 510))); // PT100
    SendToAll(str);
    sprintf(str, "M.T2.val=%d\xFF\xFF\xFF", ((int)(digitalTemp * 100)) / 10);
    SendToAll(str);
    sprintf(str, "M.Hum.val=%d\xFF\xFF\xFF", (int)digitalHum);
    SendToAll(str);
    sprintf(str, "M.Pre.val=%d\xFF\xFF\xFF", (int)digitalAlt);
    SendToAll(str);
    sprintf(str, "S.PTmv.val=%d\xFF\xFF\xFF", (int)pt100mv * 10);
    SendToAll(str);
    sprintf(str, "M.A2.val=%d\xFF\xFF\xFF", (int)a2);
    SendToAll(str);
    sprintf(str, "M.BattHourLeft.val=%d\xFF\xFF\xFF", (int)battHourLeft / 10);
    SendToAll(str);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
void sendConfig()
{
  MeasurmentTaskPause = true; // preventing sending other string
  String str = "ConfigFile=" + readStringFromFile(ConfigFile) + "END";
  Serial.println("inside:" + str);
  // SendToAll((const char *)str.c_str());
  bleSendLongString(str);
  MeasurmentTaskPause = false;
}
void MainStringProcessTask(void *parameters)
{
  char str[128];
  for (;;)
  {
    /*wait until string being received*/
    while (strlen(mainRxStr) == 0)
      vTaskDelay(1 / portTICK_PERIOD_MS);
    mainStrIsFree = false;
    /*main string process is here*/
    if (!strncmp(mainRxStr, "sw", 2)) // match only
    {
      if (lowVoltageFlg)
      {
        SendToAll("XrouteAlarm=Voltage is low !\nPlease check the battery voltage or measurement ports!\xFF\xFF\xFF");
      }
      // extract the relay number from string
      int relayNum = atoi(mainRxStr + 2);
      if ((RELAYS.relPos & (1UL << RELAYS.cnfgLookup[relayNum - 1])) == 0) //-1 is beqause of lookup table is zero based
      {
        sprintf(str, "sw%d.val=1\xFF\xFF\xFF", relayNum);
        SendToAll(str);
        RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[relayNum - 1]); //-1 is beqause of lookup table is zero based
      }
      else
      {
        sprintf(str, "sw%d.val=0\xFF\xFF\xFF", relayNum);
        SendToAll(str);
        RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[relayNum - 1]); //-1 is beqause of lookup table is zero based
      }
      setRelay(RELAYS.relPos, v / 10);
      saveStatesToFile();
    }
    else if (!strcmp(mainRxStr, "InitNextion"))
    {
      SetNextion(RELAYS.relPos, dimTmp, dimLimit);
    }
    else if (!strcmp(mainRxStr, "MOTOR1=UP"))
    {
      RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[13 - 1]);
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[14 - 1]);
      setRelay(RELAYS.relPos, v / 10);
      sprintf(str, "M1UP.val=1\xFF\xFF\xFF");
      // SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "MOTOR1=DOWN"))
    {
      RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[14 - 1]);
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[13 - 1]);
      setRelay(RELAYS.relPos, v / 10);
      sprintf(str, "M1Down.val=1\xFF\xFF\xFF");
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "MOTOR1=STOP"))
    {
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[13 - 1]);
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[14 - 1]);
      setRelay(RELAYS.relPos, v / 10);

      sprintf(str, "M1UP.val=0\xFF\xFF\xFF");
      SendToAll(str);
      sprintf(str, "M1Down.val=0\xFF\xFF\xFF");
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "MOTOR2=UP"))
    {
      RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[15 - 1]);
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[16 - 1]);
      setRelay(RELAYS.relPos, v / 10);

      sprintf(str, "M2UP.val=1\xFF\xFF\xFF");
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "MOTOR2=DOWN"))
    {
      RELAYS.relPos |= (1UL << RELAYS.cnfgLookup[16 - 1]);
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[15 - 1]);
      setRelay(RELAYS.relPos, v / 10);

      sprintf(str, "M2Down.val=1\xFF\xFF\xFF");
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "MOTOR2=STOP"))
    {
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[15 - 1]);
      RELAYS.relPos &= ~(1UL << RELAYS.cnfgLookup[16 - 1]);
      setRelay(RELAYS.relPos, v / 10);
      sprintf(str, "M1Down.val=0\xFF\xFF\xFF");
      SendToAll(str);
      sprintf(str, "M1UP.val=0\xFF\xFF\xFF"); // just for graphic reson
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "DIMER", 5) == 0) // DIMER1.val=X
    {
      float val = (float)mainRxStr[11];
      val = val / 255;
      int dimNumber = mainRxStr[5] - '0' - 1;
      dimTmp[dimNumber] = 32768 * val * dimLimit[dimNumber];
      DimValChanged = true;
      sprintf(str, "APDIM%c.val=%d\n", mainRxStr[5], mainRxStr[11]);
      SendToAll(str); /////MUST BE SEND TO BLE
    }
    else if (strncmp(mainRxStr, "APDIM", 5) == 0) // APDIM1.val=123
    {
      float val = atoi(mainRxStr + 11);
      if (val > 255 || val < 0)
        return;
      DimValChanged = true;
      dimTmp[mainRxStr[5] - '0' - 1] = 32768 * val / 255 * dimLimit[mainRxStr[5] - '0' - 1];
      sprintf(str, "DIMER%c.val=%d\xFF\xFF\xFF", mainRxStr[5], (int)val);
      if (SerialBT.connected())
        SerialBT.println(str);
    }
    else if (!strcmp(mainRxStr, "DefaultAllCalibrations"))
    {
      defaultCalibrations();
      sendAllcalibrations();
    }
    else if (strncmp(mainRxStr, "VCalTo=", 7) == 0)
    {
      VcalCo = (float)atoi(mainRxStr + 7) / volt;
      EEPROM.writeFloat(E2ADD.VcalCoSave, VcalCo);
      EEPROM.commit();
      NegVoltOffset = ADC_LPF(NEG_VOLT_MUX_IN, 5, negv, 0.99);
      EEPROM.writeFloat(E2ADD.NegVoltOffsetSave, NegVoltOffset);
      EEPROM.commit();
      sprintf(str, "show.txt=\"VcalCo=%f NegVoltOffset=%f\"\xFF\xFF\xFF", VcalCo, NegVoltOffset);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "VoltageCalibrate"))
    {
      VcalCo = (float)DFLT_V_CAL / volt;

      EEPROM.writeFloat(E2ADD.VcalCoSave, VcalCo);
      EEPROM.commit();

      NegVoltOffset = ADC_LPF(NEG_VOLT_MUX_IN, 5, negv, 0.99);
      EEPROM.writeFloat(E2ADD.NegVoltOffsetSave, NegVoltOffset);
      EEPROM.commit();

      sprintf(str, "show.txt=\"VcalCo=%f NegVoltOffset=%f\"\xFF\xFF\xFF", VcalCo, NegVoltOffset);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "VoltageCalibrate++"))
    {
      DFLT_V_CAL++;
      sprintf(str, "Vcal.val=%d\xFF\xFF\xFF", DFLT_V_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "VoltageCalibrate--"))
    {
      DFLT_V_CAL--;
      sprintf(str, "Vcal.val=%d\xFF\xFF\xFF", DFLT_V_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "AmperOffset"))
    {
      if (ampSenisConnected)
      {
        amp1Offset = amp1;
        EEPROM.writeFloat(E2ADD.ampOffsetSave, amp1Offset);
        EEPROM.commit();
        sprintf(str, "show.txt=\"amp1Offset=%f\"\xFF\xFF\xFF", amp1Offset);
        SendToAll(str);
      }
      else
      {
        SendToAll("XrouteAlarm=No External Ampermeter Detected !\xFF\xFF\xFF");
      }
    }
    else if (strncmp(mainRxStr, "ACalTo=", 7) == 0)
    {
      if (ampSenisConnected)
      {
        Serial.println(mainRxStr);
        A1calCo = (float)atoi(mainRxStr + 7) / (amp1Offset - amp1);
        EEPROM.writeFloat(E2ADD.AcalCoSave, A1calCo);
        EEPROM.commit();
        sprintf(str, "show.txt=\"A1calCo=%f\"\xFF\xFF\xFF", A1calCo);
        SendToAll(str);
      }
      else
      {
        SendToAll("XrouteAlarm=No External Ampermeter Detected !\xFF\xFF\xFF");
      }
    }
    else if (!strcmp(mainRxStr, "AmperCalibrate"))
    {
      if (ampSenisConnected)
      {
        A1calCo = DFLT_A_CAL / (amp1Offset - amp1);
        EEPROM.writeFloat(E2ADD.AcalCoSave, A1calCo);
        EEPROM.commit();
        sprintf(str, "show.txt=\"A1calCo=%f\"\xFF\xFF\xFF", A1calCo);
        SendToAll(str);
      }
      else
      {
        SendToAll("XrouteAlarm=No External Ampermeter Detected !\xFF\xFF\xFF");
      }
    }
    else if (!strcmp(mainRxStr, "AmperCalibratePlus"))
    {
      if (ampSenisConnected)
      {
        A1calCo = DFLT_A_CAL / (amp1 - amp1Offset);

        EEPROM.writeFloat(E2ADD.AcalCoSave, A1calCo);
        EEPROM.commit();
      }
      else
      {
        SendToAll("XrouteAlarm=No External Ampermeter Detected !\xFF\xFF\xFF");
      }
    }
    else if (!strcmp(mainRxStr, "AmperCalibrate++"))
    {

      DFLT_A_CAL++;
      sprintf(str, "Acal.val=%d\xFF\xFF\xFF", DFLT_A_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "AmperCalibrate--"))
    {

      DFLT_A_CAL--;
      sprintf(str, "Acal.val=%d\xFF\xFF\xFF", DFLT_A_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper0Offset"))
    {
      amp0Offset = amp0;

      EEPROM.writeFloat(E2ADD.amp0OffsetSave, amp0Offset);
      EEPROM.commit();
      sprintf(str, "show.txt=\"amp0Offset=%f\"\xFF\xFF\xFF", amp0Offset);
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "A0CalTo=", 8) == 0)
    {
      A0calCo = (float)atoi(mainRxStr + 8) / (amp0Offset - amp0);
      EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCo);
      EEPROM.commit();
      sprintf(str, "show.txt=\"A0calCo=%f\"\xFF\xFF\xFF", A0calCo);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper0Calibrate"))
    {
      A0calCo = DFLT_A0_CAL / (amp0Offset - amp0);

      EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCo);
      EEPROM.commit();
      sprintf(str, "show.txt=\"A0calCo=%f\"\xFF\xFF\xFF", A0calCo);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper0CalibratePlus"))
    {
      A0calCo = DFLT_A0_CAL / (amp0 - amp0Offset);

      EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCo);
      EEPROM.commit();
    }
    else if (!strcmp(mainRxStr, "Amper0Calibrate++"))
    {
      DFLT_A0_CAL++;
      sprintf(str, "A0cal.val=%d\xFF\xFF\xFF", DFLT_A0_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper0Calibrate--"))
    {
      DFLT_A0_CAL--;
      sprintf(str, "A0cal.val=%d\xFF\xFF\xFF", DFLT_A0_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper2Offset"))
    {
      amp2Offset = amp2;

      EEPROM.writeFloat(E2ADD.amp2OffsetSave, amp2Offset);
      EEPROM.commit();
      sprintf(str, "show.txt=\"amp2Offset=%f\"\xFF\xFF\xFF", amp2Offset);
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "A2CalTo=", 8) == 0)
    {
      A2calCo = (float)atoi(mainRxStr + 8) / (amp2Offset - amp2);
      EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCo);
      EEPROM.commit();
      sprintf(str, "show.txt=\"A2calCo=%f\"\xFF\xFF\xFF", A2calCo);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper2Calibrate"))
    {
      A2calCo = DFLT_A2_CAL / (amp2Offset - amp2);

      EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCo);
      EEPROM.commit();
      sprintf(str, "show.txt=\"A2calCo=%f\"\xFF\xFF\xFF", A2calCo);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper2CalibratePlus"))
    {
      A2calCo = DFLT_A2_CAL / (amp2 - amp2Offset);

      EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCo);
      EEPROM.commit();
    }
    else if (!strcmp(mainRxStr, "Amper2Calibrate++"))
    {
      DFLT_A2_CAL++;
      sprintf(str, "A2cal.val=%d\xFF\xFF\xFF", DFLT_A2_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Amper2Calibrate--"))
    {
      DFLT_A2_CAL--;
      sprintf(str, "A2cal.val=%d\xFF\xFF\xFF", DFLT_A2_CAL);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "BattCapCalibrate++"))
    {
      DFLT_BATT_CAP += 10;
      DFLT_BATT_CAP = constrain(DFLT_BATT_CAP, 0, 1000);
      sprintf(str, "BattCap.val=%d\xFF\xFF\xFF", DFLT_BATT_CAP);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "BattCapCalibrate--"))
    {
      DFLT_BATT_CAP -= 10;
      DFLT_BATT_CAP = constrain(DFLT_BATT_CAP, 10, 1000);
      sprintf(str, "BattCap.val=%d\xFF\xFF\xFF", DFLT_BATT_CAP);
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "BattCapCalTo=", 13) == 0)
    {
      DFLT_BATT_CAP = (float)atoi(mainRxStr + 13);
      EEPROM.writeFloat(E2ADD.batteryCapSave, DFLT_BATT_CAP);
      EEPROM.commit();
      batteryCap = DFLT_BATT_CAP;
      sprintf(str, "BattCapTxt.val=%d\xFF\xFF\xFF", (int)batteryCap);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "BattCapCalibrate"))
    {
      EEPROM.writeFloat(E2ADD.batteryCapSave, DFLT_BATT_CAP);
      EEPROM.commit();
      batteryCap = DFLT_BATT_CAP;
      sprintf(str, "BattCapTxt.val=%d\xFF\xFF\xFF", (int)batteryCap);
      SendToAll(str);
      myBattery.setBatteryCap(batteryCap);
      myBattery.setPercent(myBattery.getBtPerV());
    }
    else if (!strcmp(mainRxStr, "PTmvCalibrate++"))
    {
      DFLT_PT_MV_CAL++;
      PT_mvCal = DFLT_PT_MV_CAL / pt100;
      sprintf(str, "Pt_mvCal.val=%d\xFF\xFF\xFF", DFLT_PT_MV_CAL);
      SendToAll(str);
      if (humSensorType != HUM_SENSOR_TYPE_NON)
      {
        sprintf(str, "show.txt=\"DigitalTemp=%.1f °C\"\xFF\xFF\xFF", digitalTemp);
        SendToAll(str);
      }
    }
    else if (!strcmp(mainRxStr, "PTmvCalibrate--"))
    {
      DFLT_PT_MV_CAL--;
      PT_mvCal = DFLT_PT_MV_CAL / pt100;
      sprintf(str, "Pt_mvCal.val=%d\xFF\xFF\xFF", DFLT_PT_MV_CAL);
      SendToAll(str);
      if (humSensorType != HUM_SENSOR_TYPE_NON)
      {
        sprintf(str, "show.txt=\"DigitalTemp=%.1f °C\"\xFF\xFF\xFF", digitalTemp);
        SendToAll(str);
      }
    }
    else if (strncmp(mainRxStr, "PTCalTo=", 8) == 0)
    {
      float aimTemp = (float)atoi(mainRxStr + 8) / 10; // 231/10=23.1
      float temp;
      temp = ReadPT100_Temp(pt100mv, 510);
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
      sprintf(str, "show.txt=\"PT_mvCal=%f\"\xFF\xFF\xFF", PT_mvCal);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "PT100Calibrate"))
    {
      PT_mvCal = DFLT_PT_MV_CAL / pt100;

      EEPROM.writeFloat(E2ADD.PT_mvCal_Save, PT_mvCal);
      EEPROM.commit();
      sprintf(str, "show.txt=\"PT_mvCal=%f\"\xFF\xFF\xFF", PT_mvCal);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "BattFull+"))
    {
      DFLT_BATT_FULL_VOLT++;
      DFLT_BATT_FULL_VOLT = constrain(DFLT_BATT_FULL_VOLT, DFLT_BATT_EMPTY_VOLT, 280);
      if (DFLT_BATT_FULL_VOLT > 180 && myBattery.getBatteryArrangment() == BATTERY_CONFIG_12V)
      {
        myBattery.setBatteryArrangment(BATTERY_CONFIG_24V);
        SendToAll("XrouteAlarm=You are using 2 Battery in Series = 24v config !\xFF\xFF\xFF");
      }
      battFullVoltage = DFLT_BATT_FULL_VOLT;
      sprintf(str, "BattFullVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_FULL_VOLT);
      SendToAll(str);
      myBattery.SelectBatteryAcordingToFullVoltage(DFLT_BATT_FULL_VOLT, SendToAll);
      if (myBattery.batteryType != BATTERY_TYPE_NON)
      {
        DFLT_BATT_EMPTY_VOLT = myBattery.getBatteryEmptyVoltage() * 10 * myBattery.getBatteryArrangment();
        sprintf(str, "BattEmptyVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_EMPTY_VOLT);
        SendToAll(str);
      }
    }
    else if (!strcmp(mainRxStr, "BattFull-"))
    {
      DFLT_BATT_FULL_VOLT--;
      DFLT_BATT_FULL_VOLT = constrain(DFLT_BATT_FULL_VOLT, DFLT_BATT_EMPTY_VOLT + 10, 280);
      if (DFLT_BATT_FULL_VOLT < 180 && myBattery.getBatteryArrangment() == BATTERY_CONFIG_24V)
      {
        myBattery.setBatteryArrangment(BATTERY_CONFIG_12V);
        SendToAll("XrouteAlarm=You are using 1 Battery = 12v config !\xFF\xFF\xFF");
      }
      battFullVoltage = DFLT_BATT_FULL_VOLT;
      sprintf(str, "BattFullVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_FULL_VOLT);
      SendToAll(str);
      myBattery.SelectBatteryAcordingToFullVoltage(DFLT_BATT_FULL_VOLT, SendToAll);
      if (myBattery.batteryType != BATTERY_TYPE_NON)
      {
        DFLT_BATT_EMPTY_VOLT = myBattery.getBatteryEmptyVoltage() * 10 * myBattery.getBatteryArrangment();
        sprintf(str, "BattEmptyVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_EMPTY_VOLT);
        SendToAll(str);
      }
    }
    else if (!strcmp(mainRxStr, "BattfullVoltageCalibrate"))
    {
      EEPROM.writeFloat(E2ADD.battFullVoltageSave, DFLT_BATT_FULL_VOLT);
      EEPROM.commit();
      battFullVoltage = EEPROM.readFloat(E2ADD.battFullVoltageSave);
    }
    else if (!strcmp(mainRxStr, "BattEmpty-"))
    {
      DFLT_BATT_EMPTY_VOLT--;
      DFLT_BATT_EMPTY_VOLT = constrain(DFLT_BATT_EMPTY_VOLT, 90, DFLT_BATT_FULL_VOLT - 10);
      battEmptyVoltage = DFLT_BATT_EMPTY_VOLT;
      sprintf(str, "BattEmptyVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_EMPTY_VOLT);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "BattEmpty+"))
    {
      DFLT_BATT_EMPTY_VOLT++;
      DFLT_BATT_EMPTY_VOLT = constrain(DFLT_BATT_EMPTY_VOLT, 90, DFLT_BATT_FULL_VOLT - 10);
      battEmptyVoltage = DFLT_BATT_EMPTY_VOLT;
      sprintf(str, "BattEmptyVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_EMPTY_VOLT);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "BattEmptyVoltageCalibrate"))
    {

      EEPROM.writeFloat(E2ADD.battEmptyVoltageSave, DFLT_BATT_EMPTY_VOLT);
      EEPROM.commit();

      battEmptyVoltage = EEPROM.readFloat(E2ADD.battEmptyVoltageSave);
    }
    else if (!strncmp(mainRxStr, "BatteryType=", 12))
    {
      if (mainRxStr[12] == '1') // AGM
      {
        myBattery.setBatType(BATTERY_TYPE_AGM);
        SendToAll("XrouteAlarm= AGM BATTERY \xFF\xFF\xFF");
      }
      else if (mainRxStr[12] == '2') // GEL
      {
        myBattery.setBatType(BATTERY_TYPE_GEL);
        SendToAll("XrouteAlarm= GEL BATTERY \xFF\xFF\xFF");
      }
      else if (mainRxStr[12] == '3') // ACID
      {
        myBattery.setBatType(BATTERY_TYPE_ACID);
        SendToAll("XrouteAlarm= ACID BATTERY \xFF\xFF\xFF");
      }
      else if (mainRxStr[12] == '4') // LITHIUM
      {
        myBattery.setBatType(BATTERY_TYPE_LITIUM);
        SendToAll("XrouteAlarm= LITHIUM BATTERY \xFF\xFF\xFF");
      }
      else if (mainRxStr[12] == '5') // LIFPO4
      {
        myBattery.setBatType(BATTERY_TYPE_LIFEPO4);
        SendToAll("XrouteAlarm= LIFEPO4 BATTERY \xFF\xFF\xFF");
      }

      battFullVoltage = floor(myBattery.getBatteryFullVoltage() * 10);
      EEPROM.writeFloat(E2ADD.battFullVoltageSave, battFullVoltage);
      EEPROM.commit();
      Serial.println("battFullVoltage(X10) =" + String(battFullVoltage));
    }
    else if (!strcmp(mainRxStr, "CableRes+"))
    {
      DFLT_CABLE_RES_MILI_OHM++;
      sprintf(str, "CableRes.val=%d\xFF\xFF\xFF", DFLT_CABLE_RES_MILI_OHM);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "CableRes-"))
    {
      DFLT_CABLE_RES_MILI_OHM--;
      sprintf(str, "CableRes.val=%d\xFF\xFF\xFF", DFLT_CABLE_RES_MILI_OHM);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "CableResCalibrate"))
    {

      EEPROM.writeFloat(E2ADD.cableResistanceSave, DFLT_CABLE_RES_MILI_OHM);
      EEPROM.commit();

      cableResistance = EEPROM.readFloat(E2ADD.cableResistanceSave);
    }
    else if (!strcmp(mainRxStr, "CleanWaterMin"))
    {
      clnWtrMin = clnWtr;

      EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMin);
      EEPROM.commit();
      sprintf(str, "show.txt=\"clnWtrMin=%f\"\xFF\xFF\xFF", clnWtrMin);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "CleanWaterMax"))
    {
      clnWtrMax = clnWtr;

      EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMax);
      EEPROM.commit();
      sprintf(str, "show.txt=\"clnWtrMax=%f\"\xFF\xFF\xFF", clnWtrMax);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "DirtyWaterMin"))
    {
      drtWtrMin = drtWtr;

      EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMin);
      EEPROM.commit();
    }
    else if (!strcmp(mainRxStr, "DirtyWaterMax"))
    {
      drtWtrMax = drtWtr;

      EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMax);
      EEPROM.commit();
    }
    else if (!strcmp(mainRxStr, "GrayWaterMin"))
    {
      gryWtrMin = gryWtr;

      EEPROM.writeFloat(E2ADD.gryWtrMinSave, gryWtrMin);
      EEPROM.commit();
    }
    else if (!strcmp(mainRxStr, "GrayWaterMax"))
    {
      gryWtrMax = gryWtr;

      EEPROM.writeFloat(E2ADD.gryWtrMaxSave, gryWtrMax);
      EEPROM.commit();
    }
    else if (strncmp(mainRxStr, "LimitDim", 8) == 0)
    {
      unsigned int dimNum = mainRxStr[8] - 1;
      float val = mainRxStr[10] * 2;
      dimLimit[dimNum] = val / 255;
      dimTmp[dimNum] = (double)32767 * dimLimit[dimNum];
      DimValChanged = true;
      sprintf(str, "show.txt=\"Dim%d MaxLimit=%d\"\xFF\xFF\xFF", dimNum + 1, (int)(32768 * dimLimit[dimNum]));
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "LoadDimLimits"))
    {
      for (int i = 0; i < 7; i++)
      {
        float val = dimLimit[i] * 128;
        sprintf(str, "dimMax%d.val=%d\xFF\xFF\xFF", i + 1, (int)val);
        SendToAll(str);
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }
    else if (!strcmp(mainRxStr, "SaveDimerLimits"))
    {
      for (int i = 0; i < 7; i++)
      {
        EEPROM.writeFloat(E2ADD.dimLimitSave[i], dimLimit[i]);
        EEPROM.commit();
      }
      sprintf(str, "XrouteAlarm=Limit Saved OK! \xFF\xFF\xFF");
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "GiveMeBalance=", 14) == 0)
    {
      float ofsetlesX = (accXValue - accXValueOffset) * revX;
      float ofsetlesY = (accYValue - accYValueOffset) * revY;

      alpha = atan(ofsetlesY / ofsetlesX);

      if (ofsetlesX < 0 && ofsetlesY > 0)
        alpha += PI;
      if (ofsetlesX < 0 && ofsetlesY < 0)
        alpha += PI;
      if (ofsetlesX > 0 && ofsetlesY < 0)
        alpha += (2 * PI);
      len = sqrt(ofsetlesX * ofsetlesX + ofsetlesY * ofsetlesY) * accSensitivity;

      if (len > 1)
        len = 1;
      accSensitivity = mainRxStr[14] - 10; // to prevent \n i add 10 to slider
      accSensitivity *= 2;

      sprintf(str, "Accx.val=%d\xFF\xFF\xFF", (int)(roundf(len * cos(alpha) * 100)));
      SendToAll(str);
      sprintf(str, "Accy.val=%d\xFF\xFF\xFF", (int)(roundf(len * sin(alpha) * 100)));
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "AccelZeroOffset"))
    {
      GyroOffsetingFlg = true;
      while (GyroOffsetingFlg)
        vTaskDelay(10 / portTICK_PERIOD_MS);
      EEPROM.writeFloat(E2ADD.accXValueOffsetSave, accXValue);
      EEPROM.writeFloat(E2ADD.accYValueOffsetSave, accYValue);
      EEPROM.commit();
      accXValueOffset = EEPROM.readFloat(E2ADD.accXValueOffsetSave);
      accYValueOffset = EEPROM.readFloat(E2ADD.accYValueOffsetSave);

      sprintf(str, "XrouteAlarm=accXValueOffset=%f,accYValueOffset=%f\xFF\xFF\xFF", accXValueOffset, accYValueOffset);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "GiveMeSysInfo"))
    {
      uint64_t chipid = ESP.getEfuseMac();
      sprintf(str, "MacAddress:%012llX,%s,Version:%s\xFF\xFF\xFF", chipid, GeneralLisence.c_str(), Version);
      Serial.println(str);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "GyroPass:deactive"))
    {
      GyroLicense->deactivate();
    }
    else if (strncmp(mainRxStr, "GyroPass:", 9) == 0)
    {
      char tmp[17];
      for (int i = 0; i < 16; i++)
      {
        tmp[i] = mainRxStr[9 + i];
      }
      tmp[16] = '\0';
      sprintf(str, "ReceivedPass:%s", tmp);
      Serial.println(str);
      sprintf(str, "InternalPass:%s", GyroLicense->realSerial);
      Serial.println(str);

      if (strcmp(GyroLicense->realSerial, tmp) == 0)
      {
        GyroLicense->activate();
      }
    }
    else if (strncmp(mainRxStr, "GyroOrientation=", 16) == 0)
    {
      GyroOriantation.setCharAt(0, mainRxStr[16]);
      GyroOriantation.setCharAt(1, mainRxStr[17]);
      GyroOriantation.setCharAt(2, mainRxStr[18]);
      GyroOriantation.setCharAt(3, mainRxStr[19]);
      GyroOriantation.setCharAt(4, '\0');
      // Serial.println("BEFOR SAVE ORIANTATION:" + GyroOriantation);
      EEPROM.writeString(E2ADD.GyroOriantationSave, GyroOriantation);
      EEPROM.commit();
      char strTmp[32];
      EEPROM.readString(E2ADD.GyroOriantationSave, strTmp, 16);
      GyroOriantation = String(strTmp);
      // Serial.println("AFTER SAVE ORIANTATION:" + GyroOriantation);
    }
    else if (!strcmp(mainRxStr, "GiveMeOrientation"))
    {
      uint64_t chipid = ESP.getEfuseMac();
      sprintf(str, "Orientation=%s\xFF\xFF\xFF", GyroOriantation);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "PreCalibrate"))
    {
      EEPROM.writeFloat(E2ADD.pressurCalOffsetSave, pressurCalOffset);
      EEPROM.commit();
      sprintf(str, "show.txt=\"PresOffset=%f\"\xFF\xFF\xFF", pressurCalOffset);
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Pre+"))
    {
      pressurCalOffset += 0.01;
      sprintf(str, "Pcal.val=%d\xFF\xFF\xFF", (int)(pressurCalOffset * 100));
      SendToAll(str);
    }
    else if (!strcmp(mainRxStr, "Pre-"))
    {
      pressurCalOffset -= 0.01;
      sprintf(str, "Pcal.val=%d\xFF\xFF\xFF", (int)(pressurCalOffset * 100));
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "PreCalTo=", 9) == 0)
    {
      float aimAlt = (float)atoi(mainRxStr + 9); // altitute in meters
      float tempAlt = psiToMeters(BARO.readPressure(PSI) - pressurCalOffset);
      float error = 1; // to get into the loop
      float pressurCalOffsetTemp = pressurCalOffset;
      sprintf(str, "show.txt=\"Calibrating=\"\xFF\xFF\xFF");
      SendToAll(str);

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
        sprintf(str, "M.Pre.val=%d\xFF\xFF\xFF", (int)tempAlt);
        SendToAll(str);
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      if (!isnan(pressurCalOffsetTemp)) // for somthimes crashes
      {
        pressurCalOffset = pressurCalOffsetTemp;
        EEPROM.writeFloat(E2ADD.pressurCalOffsetSave, pressurCalOffset);
        EEPROM.commit();
      }
      sprintf(str, "show.txt=\"pressurCalOffset=%f\"\xFF\xFF\xFF", pressurCalOffset);
      SendToAll(str);
    }
    else if (strncmp(mainRxStr, "BLEPASSWORD=", 12) == 0)
    {
      char pass[7];
      strncpy(pass, mainRxStr + 12, 6);
      pass[6] = '\0'; // Corrected null terminator assignment
      blePass = atoi(pass);
      Serial.print("Password=");
      Serial.println(blePass);
      EEPROM.writeUInt(E2ADD.blePassSave, blePass);
      EEPROM.commit();
      bleSetPass(blePass);
      remove_all_bonded_devices();
      //      ESP.restart();
    }
    else if (!strcmp(mainRxStr, "GETBLEPASSWORD"))
    {
      char str[32];
      blePass = EEPROM.readUInt(E2ADD.blePassSave);
      sprintf(str, "BLEPASSWORD=%d\xFF\xFF\xFF", blePass);
      SendToAll(str);
      Serial.println(str);
    }
    else if (!strncmp(mainRxStr, "LPM=", 4))
    {
      String str = String(mainRxStr).substring(4);
      String dimStr = str.substring(0, str.indexOf(","));
      String relStr = str.substring(str.indexOf(",") + 1, str.lastIndexOf(","));
      String volStr = str.substring(str.lastIndexOf("=") + 1, str.length());
      BatteryLowPrcnt = volStr.toFloat() * 10; // 10 times is for the reason that all my code is 10 times biger
      BatteryLowPrcntRelays = strtoul(relStr.c_str(), NULL, 2);
      BattLowPrcntDimers = strtoul(dimStr.c_str(), NULL, 2);
      EEPROM.writeFloat(E2ADD.lowVoltageSave, BatteryLowPrcnt);
      EEPROM.writeInt(E2ADD.lowVoltageRelaysSave, BatteryLowPrcntRelays);
      EEPROM.writeInt(E2ADD.lowVoltageDimersSave, BattLowPrcntDimers);
      EEPROM.commit();
      char str1[64];
      Serial.println(mainRxStr);
      sprintf(str1, "XrouteAlarm=Low Voltage Parameters Set succesfully!\xFF\xFF\xFF");
      SendToAll(str1);
    }
    else if (!strncmp(mainRxStr, "CPM=", 4))
    {
      String str = String(mainRxStr).substring(4);
      String dimStr = str.substring(0, str.indexOf(","));
      String relStr = str.substring(str.indexOf(",") + 1, str.lastIndexOf(","));
      String volStr = str.substring(str.lastIndexOf("=") + 1, str.length());
      BattCriticalPrcnt = volStr.toFloat() * 10; // 10 times is for the reason that all my code is 10 times biger
      BattCriticalPrcntRelays = strtoul(relStr.c_str(), NULL, 2);
      BattCriticalPrcntDimers = strtoul(dimStr.c_str(), NULL, 2);
      EEPROM.writeFloat(E2ADD.criticalVoltageSave, BattCriticalPrcnt);
      EEPROM.writeInt(E2ADD.criticalVoltageRelaysSave, BattCriticalPrcntRelays);
      EEPROM.writeInt(E2ADD.criticalVoltageDimersSave, BattCriticalPrcntDimers);
      EEPROM.commit();
      char str1[64];
      Serial.println(mainRxStr);
      sprintf(str1, "XrouteAlarm=Critical Voltage Parameters Set succesfully\xFF\xFF\xFF");
      SendToAll(str1);
    }
    else if (!strncmp(mainRxStr, "GiveMeLPM", 9))
    {
      char str[64];
      sprintf(str, "LPM=%d%d%d%d%d%d%d,%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d,V=%1.f\xFF\xFF\xFF",
              bitRead(BattLowPrcntDimers, 6), bitRead(BattLowPrcntDimers, 5), bitRead(BattLowPrcntDimers, 4), bitRead(BattLowPrcntDimers, 3),
              bitRead(BattLowPrcntDimers, 2), bitRead(BattLowPrcntDimers, 1), bitRead(BattLowPrcntDimers, 0), bitRead(BatteryLowPrcntRelays, 15),
              bitRead(BatteryLowPrcntRelays, 14), bitRead(BatteryLowPrcntRelays, 13), bitRead(BatteryLowPrcntRelays, 12), bitRead(BatteryLowPrcntRelays, 11),
              bitRead(BatteryLowPrcntRelays, 10), bitRead(BatteryLowPrcntRelays, 9), bitRead(BatteryLowPrcntRelays, 8), bitRead(BatteryLowPrcntRelays, 7),
              bitRead(BatteryLowPrcntRelays, 6), bitRead(BatteryLowPrcntRelays, 5), bitRead(BatteryLowPrcntRelays, 4), bitRead(BatteryLowPrcntRelays, 3),
              bitRead(BatteryLowPrcntRelays, 2), bitRead(BatteryLowPrcntRelays, 1), bitRead(BatteryLowPrcntRelays, 0), BatteryLowPrcnt);
      SendToAll(str);
      Serial.println(str);
    }
    else if (!strncmp(mainRxStr, "GiveMeCPM", 9))
    {
      char str[64];
      sprintf(str, "CPM=%d%d%d%d%d%d%d,%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d,V=%1.f\xFF\xFF\xFF",
              bitRead(BattCriticalPrcntDimers, 6), bitRead(BattCriticalPrcntDimers, 5), bitRead(BattCriticalPrcntDimers, 4), bitRead(BattCriticalPrcntDimers, 3),
              bitRead(BattCriticalPrcntDimers, 2), bitRead(BattCriticalPrcntDimers, 1), bitRead(BattCriticalPrcntDimers, 0), bitRead(BattCriticalPrcntRelays, 15),
              bitRead(BattCriticalPrcntRelays, 14), bitRead(BattCriticalPrcntRelays, 13), bitRead(BattCriticalPrcntRelays, 12), bitRead(BattCriticalPrcntRelays, 11),
              bitRead(BattCriticalPrcntRelays, 10), bitRead(BattCriticalPrcntRelays, 9), bitRead(BattCriticalPrcntRelays, 8), bitRead(BattCriticalPrcntRelays, 7),
              bitRead(BattCriticalPrcntRelays, 6), bitRead(BattCriticalPrcntRelays, 5), bitRead(BattCriticalPrcntRelays, 4), bitRead(BattCriticalPrcntRelays, 3),
              bitRead(BattCriticalPrcntRelays, 2), bitRead(BattCriticalPrcntRelays, 1), bitRead(BattCriticalPrcntRelays, 0), BattCriticalPrcnt);
      SendToAll(str);
      Serial.println(str);
    }
    else if (!strncmp(mainRxStr, "DEF=", 4))
    {
      if (!strcmp(mainRxStr, "DEF=VOLTAGE"))
      {
        EEPROM.writeFloat(E2ADD.VcalCoSave, VcalCoDeflt);
        EEPROM.writeFloat(E2ADD.NegVoltOffsetSave, NegVoltOffsetDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=A0"))
      {
        EEPROM.writeFloat(E2ADD.amp0OffsetSave, amp0OffsetDeflt);
        EEPROM.writeFloat(E2ADD.A0calCoSave, A0calCoDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=A"))
      {
        EEPROM.writeFloat(E2ADD.ampOffsetSave, ampOffsetDeflt);
        EEPROM.writeFloat(E2ADD.AcalCoSave, AcalCoDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=A2"))
      {
        EEPROM.writeFloat(E2ADD.A2calCoSave, A2calCoDeflt);
        EEPROM.writeFloat(E2ADD.amp2OffsetSave, amp2OffsetDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=PT"))
      {
        EEPROM.writeFloat(E2ADD.PT_mvCal_Save, PT_mvCal_Deflt);
      }
      else if (!strcmp(mainRxStr, "DEF=CW_MIN"))
      {
        EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMaxDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=CW_MAX"))
      {
        EEPROM.writeFloat(E2ADD.clnWtrMinSave, clnWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.clnWtrMaxSave, clnWtrMaxDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=DW_MIN"))
      {
        EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMaxDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=DW_MAX"))
      {
        EEPROM.writeFloat(E2ADD.drtWtrMinSave, drtWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.drtWtrMaxSave, drtWtrMaxDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=GW_MIN"))
      {
        EEPROM.writeFloat(E2ADD.gryWtrMinSave, gryWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.gryWtrMaxSave, gryWtrMaxDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=GW_MAX"))
      {
        EEPROM.writeFloat(E2ADD.gryWtrMinSave, gryWtrMinDeflt);
        EEPROM.writeFloat(E2ADD.gryWtrMaxSave, gryWtrMaxDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=DIMMER")) // limits
      {
        for (int i = 0; i < 7; i++) // save dimmer defaults
        {
          EEPROM.writeFloat(E2ADD.dimLimitSave[i], dimLimitDeflt); // limit dimers to %60
        }
      }
      else if (!strcmp(mainRxStr, "DEF=ALTITUDE"))
      {
        EEPROM.writeFloat(E2ADD.pressurCalOffsetSave, pressurCalOffsetDeflt);
      }
      else if (!strcmp(mainRxStr, "DEF=GAS_MIN"))
      {
        /* code */
      }
      else if (!strcmp(mainRxStr, "DEF=GAS_MAX"))
      {
        /* code */
      }
      EEPROM.commit();
      loadSavedValue();
      SendToAll("XrouteAlarm=Default OK\xFF\xFF\xFF");
    }
    else if (!strncmp(mainRxStr, "StartUpdate=", 12))
    {
      // Get flash chip size in bytes
      uint32_t flashSize = ESP.getFlashChipSize();
      // Convert flash size from bytes to megabytes
      float flashSizeMB = (float)flashSize / (1024.0 * 1024.0);
      Serial.println("-----Flash info-----"); // for padding problem i altered the file size . change it if the update goes to 100% and not finish
      Serial.print("FlashSize:");
      Serial.print(flashSizeMB);
      Serial.println("MB");
      if (flashSizeMB > 15)
      {
        char str[16];
        uint8_t aknoledge[] = {'U', 'P', 'D', 0xff, 0xff, 0xff};
        UpdatingFlg = true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        unsigned long time;
        unsigned long timeout;

        SerialBT.setTimeout(1000);
        String strTmp = mainRxStr;
        strTmp = strTmp.substring(12, strTmp.indexOf("Bytes"));
        updateLen = atoi(strTmp.c_str());
        SerialBT.println("Update Received : " + String(updateLen) + " Bytes\xFF\xFF\xFF");
        Serial.println("Update Received : " + String(updateLen) + " Bytes\xFF\xFF\xFF");
        Update.begin(updateLen);

        long progress = 0;
        int chunkCntr = updateLen / CHUNK_SIZE;
        int byteCntr = updateLen % CHUNK_SIZE;
        int prgrs = 0;
        int lastPrgrs = 0;
        time = millis();
        for (int i = 0; i < chunkCntr; i++)
        {
          timeout = millis();
          SerialBT.write(aknoledge, 6);
          SerialBT.flush();

          SerialBT.readBytes(dataBuff, CHUNK_SIZE);
          if ((millis() - timeout) > 900)
          {
            SerialBT.println("TimeOut happend! UpdateFailed. Restaring...\xFF\xFF\xFF");
            Serial.println("TimeOut happend! UpdateFailed. Restaring...\xFF\xFF\xFF");
            SerialBT.flush();
            Serial.flush();
            ESP.restart();
          }
          aes.decrypt(dataBuff, CHUNK_SIZE, dataBuff, key, sizeof(key), iv);
          Update.write(dataBuff, CHUNK_SIZE);
          if (((i * CHUNK_SIZE) % 4096) == 0)
          {
            prgrs = Update.progress() * 100 / updateLen;
            if (prgrs > lastPrgrs)
            {
              sprintf(str, "PRGU=%d\xFF\xFF\xFF", prgrs);
              SerialBT.println(str);
              Serial.println(str);
            }
            lastPrgrs = prgrs;
          }
        }
        SerialBT.write(aknoledge, 6);
        SerialBT.flush();

        SerialBT.readBytes(dataBuff, byteCntr);
        // Decrypt the remaining bytes
        aes.decrypt(dataBuff, byteCntr, dataBuff, key, sizeof(key), iv);
        Update.write(dataBuff, byteCntr);
        SerialBT.println(Update.progress());
        if (Update.end() == true)
        {
          time = millis() - time;
          sprintf(str, "PRGU=100\xFF\xFF\xFF");
          SerialBT.println(str);
          Serial.println(str);
          SerialBT.flush();
          Serial.flush();
          SerialBT.println("Update Successful in : (" + String(time / 1000) + ") Sec\xFF\xFF\xFF");
          Serial.println("Update Successful in : (" + String(time / 1000) + ") Sec\xFF\xFF\xFF");
          SerialBT.flush();
          Serial.flush();
          SerialBT.println("Restarting in \xFF\xFF\xFF");
          Serial.println("Restarting in \xFF\xFF\xFF");
          SerialBT.flush();
          Serial.flush();
          for (int i = 5; i > 0; i--)
          {
            SerialBT.println(i);
            Serial.println(i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            SerialBT.flush();
            Serial.flush();
          }
          ESP.restart();
        }
        else
        {
          SerialBT.print("\nFailed !\xFF\xFF\xFF");
          SerialBT.println(Update.errorString());
        }
      }
      else
      {
        sprintf(str, "PRGU=100\xFF\xFF\xFF");
        SerialBT.println(str);
        sprintf(str, "XrouteAlarm= Your Device Memory is %dMB and Dose Not Support Update !", flashSizeMB);
        SendToAll(str);
        SerialBT.print("\nFailed !\xFF\xFF\xFF");
        SerialBT.println(Update.errorString());
      }
    }
    else if (!strncmp(mainRxStr, "TakeUiConfig=", 13))
    {
      int configLen;
      uint8_t aknoledge[] = {'U', 'P', 'D', 0xff, 0xff, 0xff};
      //        SerialBT.write(aknoledge, 6);
      MeasurmentTaskPause = true;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      SerialBT.setTimeout(1000);
      String strTmp = mainRxStr;
      strTmp = strTmp.substring(13, strTmp.indexOf("Bytes"));
      configLen = atoi(strTmp.c_str());
      SerialBT.println("Update Received : " + String(configLen) + " Bytes\xFF\xFF\xFF");
      Serial.println("Config File Size: " + String(configLen) + " Bytes\xFF\xFF\xFF");
      Serial.flush();
      int chunkCntr = configLen / CHUNK_SIZE;
      int byteCntr = configLen % CHUNK_SIZE;
      int dataSize;
      confAndCondStrBuffer.clear();
      for (int i = 0; i < chunkCntr; i++)
      {
        SerialBT.write(aknoledge, 6);
        SerialBT.flush();
        dataSize = SerialBT.readBytes(dataBuff, CHUNK_SIZE);
        for (int i = 0; i < dataSize; i++)
        {
          confAndCondStrBuffer += (char)dataBuff[i];
        }
      }
      SerialBT.write(aknoledge, 6);
      SerialBT.flush();

      SerialBT.readBytes(dataBuff, byteCntr);
      for (int i = 0; i < byteCntr; i++)
      {
        confAndCondStrBuffer += (char)dataBuff[i];
      }
      confAndCondStrBuffer[configLen] = '\0';
      SaveStringToFile(confAndCondStrBuffer, ConfigFile);
      Serial.flush();
      confAndCondStrBuffer.clear();
      MeasurmentTaskPause = false;
      sendConfig();
    }
    else if (strcmp(mainRxStr, "GiveMeConfigFile") == 0)
    {
      sendConfig();
    }
    else
    {
      Serial.println(mainRxStr);
    }
    /*main string process ends here*/
    mainRxStr[0] = '\0'; // delet main string
    mainStrIsFree = true;
  }
}
void stringHandelingTask(void *parameters)
{
  static String str;
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);

    /*if (BLE_DATA.RxDataReadyFlag)
    {
      // wait untill main process on str finished
      while (mainStrIsFree == false)
        vTaskDelay(1 / portTICK_PERIOD_MS);
      strcpy(mainRxStr, BLE_DATA.bleRxStr);
      // Serial.printf("mainRxStr=%s", mainRxStr);
      BLE_DATA.RxDataReadyFlag = false;
    }*/
    // this is the new parser added to convert a single compound message to multiple commands : 23/5/2024
    if (BLE_DATA.RxDataReadyFlag)
    {
      String str = BLE_DATA.bleRxStr;
      //      str=str.substring(0,str.lastIndexOf('\n'));
      while (1)
      {
        int indx = str.indexOf('\n');
        if (indx < 1)
          break;
        String Str2 = str.substring(0, indx);
        str = str.substring(indx + 1);
        while (mainStrIsFree == false || strlen(mainRxStr) != 0)
          vTaskDelay(pdMS_TO_TICKS(1));
        strcpy(mainRxStr, Str2.c_str());
        // Serial.printf("mainRxStr=%s", mainRxStr);
      }
      BLE_DATA.RxDataReadyFlag = false;
    }

    if (SerialBT.connected())
    {
      while (SerialBT.available())
      {
        while (mainStrIsFree == false || strlen(mainRxStr) != 0) ////////
          vTaskDelay(10 / portTICK_PERIOD_MS);                   //////???
        mainStrIsFree = false;                                   /////////
        str = SerialBT.readStringUntil('\n');
        // Serial.println("|"+str+"|");
        for (int i = 0; i < str.length(); i++)
        {
          mainRxStr[i] = str[i];
        }
        mainRxStr[str.length()] = '\0';
        mainStrIsFree = true; //////////
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void BLE_TASK(void *parameters)
{
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);

    BLEloop();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
void DimerTask(void *parameters)
{
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
        DimValChanged = false;

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
  int cntr = 0;
  bool frsTime = true;
  bool frsTime2 = true;

  for (;;)
  {
    if (UpdatingFlg)
    {
      strip.setBrightness(250);
    }
    while (UpdatingFlg)
    {
      ws2812Blink(COLOR_WHITE);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    if (overCrntFlg == true)
    {
      ws2812Blink(COLOR_ORANG);
    }
    else if (SerialBT.connected() == true)
    {
      if (frsTime == true)
      {
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        frsTime = false;
        frsTime2 = true;
        Serial.println("ESP_BT_NON_DISCOVERABLE");
      }
      ws2812Blink(COLOR_GREEN);
    }
    else
    {
      if (frsTime2 == true)
      {
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        Serial.println("ESP_BT_DISCOVERABLE");
        frsTime2 = false;
        frsTime = true;
      }
    }
    if (BLE_DATA.deviceConnected == true)
      ws2812Blink(COLOR_BLUE);
    if (SerialBT.connected() == false && BLE_DATA.deviceConnected == false)

      ws2812Blink(COLOR_RED);
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
      sprintf(str, "DIMER%d.val=%d\xFF\xFF\xFF", dimShortNum + 1, 0);
      SendToAll(str);
      sprintf(str, "XrouteAlarm= PROTECTION! Over current at DIMMER : %d\xFF\xFF\xFF", dimShortNum + 1);
      SendToAll(str);
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
          sprintf(str, "XrouteAlarm=OVER CURRENT protection. Amp > 14.0 \xFF\xFF\xFF");
          SendToAll(str);
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
void inline WaitForStrQueToFinish()
{
  while (mainStrIsFree == false || strlen(mainRxStr) > 0)
  {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void LOWPOWER_CONTROL_TASK(void *parameters)
{
  char str[32];
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  Serial.println("Low Voltage Protaction Activated !");
  for (;;)
  {
    if (UpdatingFlg)
      vTaskDelete(NULL);
    // Serial.println("Percent:" + String(b) + "LpPrcnt:" + String(BatteryLowPrcnt / 10) + "BattCriticalPrcnt" + String(BattCriticalPrcnt / 10));
    if (b < BatteryLowPrcnt / 10 && b > BattCriticalPrcnt / 10)
    {
      static boolean flg = false;
      for (int i = 0; i < 7; i++)
      {
        if (bitRead(BattLowPrcntDimers, i) == true && dimTmp[i] > 10)
        {
          flg = true;
          dimLPF[i] = 0;
          sprintf(str, "APDIM%d.val=0\n", i + 1);
          WaitForStrQueToFinish();
          sendCmndToMainStringProcessorTask(str);
          sprintf(str, "DIMER%d.val=\x00\n", i + 1);
          WaitForStrQueToFinish();
          sendCmndToMainStringProcessorTask(str);
          Serial.println(str);
        }
      }
      for (int i = 0; i < 16; i++)
      {
        if (bitRead(BatteryLowPrcntRelays, i) == true && relState_0_15(i) == true)
        {
          flg = true;
          sprintf(str, "sw%d\n", i + 1);
          WaitForStrQueToFinish();
          sendCmndToMainStringProcessorTask(str);
          Serial.println(str);
        }
      }
      if (flg)
      {
        SendToAll("XrouteAlarm=Voltage is low ! please check the battery voltage measurement ports\xFF\xFF\xFF");
        flg = false;
      }
    }
    else if (b < BatteryLowPrcnt / 10 && b < BattCriticalPrcnt / 10)
    {
      static boolean flg = false;
      for (int i = 0; i < 7; i++)
      {
        if (bitRead(BattCriticalPrcntDimers, i) == true && dimTmp[i] > 10)
        {
          flg = true;
          dimLPF[i] = 0;
          sprintf(str, "APDIM%d.val=0\n", i + 1);
          WaitForStrQueToFinish();
          sendCmndToMainStringProcessorTask(str);
          sprintf(str, "DIMER%d.val=\x00\n", i + 1);
          // sendCmndToMainStringProcessorTask(str);
          Serial.println(str);
        }
      }
      for (int i = 0; i < 16; i++)
      {
        if (bitRead(BattCriticalPrcntRelays, i) == true && relState_0_15(i) == true)
        {
          flg = true;
          sprintf(str, "sw%d\n", i + 1);
          WaitForStrQueToFinish();
          sendCmndToMainStringProcessorTask(str);
          Serial.println(str);
        }
      }
      if (flg)
      {
        SendToAll("XrouteAlarm=Voltage is in critical level !\xFF\xFF\xFF");
        flg = false;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
        SendToAll("XrouteAlarm=Amp Meter Connected !\xFF\xFF\xFF");
        ampSenisConnected = true;
        myBattery.setPercent(myBattery.getBtPerV());
      }
      b = myBattery.getPercent() * 100;
    }
    else
    {
      if (ampSenisConnected)
      {
        SendToAll("XrouteAlarm=Amp Meter Disconnected !\xFF\xFF\xFF");
        ampSenisConnected = false;
      }
      b = constrain(myBattery.btPerV, 0, 1) * 100;
    }
    // Serial.println(b);

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
    sprintf(str, "XrouteAlarm=Default Saved OK !\xFF\xFF\xFF");
    SendToAll(str);
    loadSavedValue();
    for (int i = 0; i < 7; i++) // load dimmer defaults
    {
      dimLimit[i] = EEPROM.readFloat(E2ADD.dimLimitSave[i]);
    }
    for (int i = 0; i < 7; i++) // show dimmer defaults
    {
      float val = dimLimit[i] * 128;
      sprintf(str, "dimMax%d.val=%d\xFF\xFF\xFF", i + 1, (int)val);
      SendToAll(str);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
  else
  {
    sprintf(str, "XrouteAlarm=SAVE ERROR !\xFF\xFF\xFF");
    SendToAll(str);
  }
}
void sendCmndToMainStringProcessorTask(char *str)
{
  while (mainStrIsFree == false)
  {
    vTaskDelay(pdTICKS_TO_MS(1));
  }
  mainStrIsFree = false;
  strcpy(mainRxStr, str);
  mainStrIsFree = true;
}
// END----------------------------------------------TASKS
//-------------------------------------------------FUNCTIONS
void generate433(uint16_t *Times)
{
#define SigPin 22
#define SIG_HIGH() digitalWrite(SigPin, 1)
#define SIG_LOW() digitalWrite(SigPin, 0)
#define SIG_TOGGLE() digitalWrite(SigPin, !digitalRead(SigPin))
  int edgCntr = 0;
  while (Times[edgCntr] != 0)
    edgCntr++;

  pinMode(SigPin, OUTPUT);
  SIG_HIGH();
  bool flg = 1;
  for (int i = 0; i < edgCntr; i++)
  {
    unsigned long time;
    time = micros();
    while ((micros() - time) < (Times[i] - 1))
    {
    }
    if (i != (edgCntr - 1))
      flg = !flg;
    digitalWrite(SigPin, flg);
  }
}
int getRemoteLen(uint16_t *data)
{
  int cnt = 0;
  while (data[cnt] != 0)
    cnt++;
  return cnt;
}
void saveRemoteKey(uint16_t *times, int key)
{
  int count = getRemoteLen(times);
  for (int i = 0; i < count; i++)
  {
    EEPROM.writeUInt(E2ADD.remoteKeysAddress[key + i * 2], times[i]);
  }
  EEPROM.commit();
}
void loadRemoteKey(uint16_t *times, int key)
{
  int i = 0;
  do
  {
    times[i] = EEPROM.readUInt(E2ADD.remoteKeysAddress[key + i * 2]);
  } while (times[i++] != 0);
}
void timeMeasure(uint16_t *keyData)
{
#define DataPin 27
#define TimeOutUs 100000
#define MaxTimeTorolant 0.75
#define SIZE (64 * 2)
  pinMode(DataPin, INPUT);
#define WaitFor_1() while (!digitalRead(DataPin))
#define WaitFor_0() while (digitalRead(DataPin))
  Serial.println("Press Key : o");
  while (1)
  {
    if (Serial.read() == 'o')
    {
      break;
    }
  }
  unsigned long maxTime = 0;
  //===============================Max time measure
  uint16_t Times[SIZE];
  WaitFor_1();
  WaitFor_0();
  for (int i = 0; i < SIZE; i++)
  {
    unsigned long time = micros();
    WaitFor_1();
    time = micros() - time;
    if (time > TimeOutUs)
    {
      return;
    }

    if (time > maxTime)
      maxTime = time;
    WaitFor_0();
  }
  Serial.println("Max Time: ");
  Serial.println(maxTime);
  //==============================waite for maxtime to happen
  WaitFor_0();
  WaitFor_1();
  while (1)
  {
    unsigned long time;
    unsigned long t1;
    if (digitalRead(DataPin))
    {
      t1 = micros();
    }
    time = micros() - t1;
    if (time > (maxTime * MaxTimeTorolant))
    {
      break;
    }
  }
  //==============================count Bits and TimeSave
  int bitCnt = 0;
  int edgCntr = 0;
  while (1)
  {
    unsigned long lowTime, t1L;
    unsigned long highTime, t1h;
    WaitFor_1();
    t1h = micros();
    WaitFor_0();
    Times[edgCntr++] = micros() - t1h;
    t1L = micros();
    bitCnt++;
    if (bitCnt > SIZE)
    {
      Serial.println("Error in Bit Size");
      return;
    }

    WaitFor_1()
    {
      lowTime = micros() - t1L;
      if (lowTime > (maxTime * MaxTimeTorolant))
        break;
    }
    Times[edgCntr++] = micros() - t1L;

    if (lowTime > (maxTime * MaxTimeTorolant))
      break;
  }
  Times[edgCntr - 1] = maxTime;
  Times[edgCntr] = 0; // end of data
                      //=====================================returns valid datas
  for (int i = 0; i < edgCntr; i++)
  {
    keyData[i] = Times[i];
  }
  return; // END
  //========================================GenerateSignal
  WaitFor_1();
  while (1)
  {
    if (Serial.read() == 's')
    {
      Serial.println("Sending 433");
      for (int i = 0; i < 10; i++)
      {
        generate433(Times);
      }
      Serial.println("Done");
    }
    if (Serial.read() == 'a')
    {
      break;
    }
  }
}
void setup433()
{
  Serial.begin(115200);
  Serial.println("Starting...");
  EEPROM.begin(8704);

  while (1)
  {
    uint16_t key1Data[64 * 2];
    char c = Serial.read();

    if (c == 'r')
    {
      Serial.println("Start Decoding ...");
      timeMeasure(key1Data);
      Serial.println("Finished decoding.");
    }
    if (c == 's')
    {
      Serial.println("Save Start");
      saveRemoteKey(key1Data, 0);
      Serial.println("Save Finish");
    }
    if (c == 'l')
    {
      Serial.println("Load Start");
      loadRemoteKey(key1Data, 0);
      Serial.println("Load Finish");
    }
    if (c == 'a')
    {
      Serial.println("Sending Start");
      for (int i = 0; i < 10; i++)
      {
        generate433(key1Data);
      }
      Serial.println("Sendin Finish");
    }
  }
}
void createCondition(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue)
{
  cndtions.push_back(Conditions(_inputType, _inputPort, _oprt, _setpoint, _outputType, _outputPort, _outputValue)); // 0
}

void setup()
{
  Serial.begin(115200);
  initRelay();
  initLED_PWM();
  if (SPIFFS.begin(true))
  {
    Serial.println("SPIFF OK !");
    loadStateFromFile();
    // SaveStringToFile(String(defaultConfig), ConfigFile); // for test and it should be removed
    // UI Config File
    if (!SPIFFS.exists(ConfigFile))
    {
      if (!SaveStringToFile(String(defaultConfig), ConfigFile))
      {
        Serial.println("Error Saving: " + String(ConfigFile));
      }
    }
    String fileContent = readStringFromFile(ConfigFile);
    Serial.println("Config File Content: " + fileContent);
    // CONDITIONS
    if (!SPIFFS.exists(CondFile))
    {
      if (!SaveStringToFile(String(defaultCondition), CondFile))
      {
        Serial.println("Error Saving: " + String(CondFile));
      }
    }
    fileContent = readStringFromFile(CondFile);
    Serial.println("Condition File Content: " + fileContent);
  }
  else
  {
    Serial.println("SPIFF ERROR !");
  }

  conditionSetVariables(&v, &a0, &a1, &w, &b, &cwPrcnt, &dwPrcnt, &gwPrcnt, &digitalTemp, &digitalHum, &digitalAlt, &pt100, &a2, &battHourLeft);
  setCmdFunction(&sendCmndToMainStringProcessorTask);
  getRelayStateFunction(&relState_0_15);
  getDimValueFunction(&getDimVal);
  setcondCreatorFunction(&createCondition);
  jsonCon.readJsonConditionsFromFile(CondFile);

  // cndtions.push_back(Conditions("FLT", 0, "<", 700, "DIM", 1, 10)); // 1

  // cndtions.push_back(Conditions("VOL", 0, "<", 130, "REL", 1, 0));  // 2
  // cndtions.push_back(Conditions("VOL", 0, "<", 130, "DIM", 2, 0));  // 3
  // cndtions.push_back(Conditions("VOL", 0, ">", 130, "REL", 1, 1));  // 4
  // cndtions.push_back(Conditions("VOL", 0, ">", 130, "DIM", 2, 20)); // 5

  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 1, 1)); // 6
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 2, 1)); // 7
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 3, 1)); // 8
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 4, 1)); // 9
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 5, 1)); // 10
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 6, 1)); // 11
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 7, 1)); // 12
  // cndtions.push_back(Conditions("REL", 9, "==", 1, "REL", 8, 1)); // 13

  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 1, 0)); // 14
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 2, 0)); // 15
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 3, 0)); // 16
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 4, 0)); // 17
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 5, 0)); // 18
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 6, 0)); // 19
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 7, 0)); // 20
  // cndtions.push_back(Conditions("REL", 10, "==", 1, "REL", 8, 0)); // 21

  // cndtions.push_back(Conditions("AMP", 2, ">", 20, "DIM", 3, 200)); // 22
  // cndtions.push_back(Conditions("AMP", 2, ">", 20, "DIM", 3, 0));  // 23 blinking
  // cndtions.push_back(Conditions("AMP", 2, "<", 20, "DIM", 3, 0)); // 24

  // cndtions.push_back(Conditions("DIM", 4, ">", 100, "DIM", 5, 150)); // 24
  // cndtions.push_back(Conditions("DIM", 4, "<", 100, "DIM", 5, 0)); // 24

  // cndtions.push_back(Conditions("HUM", 0, ">", 45, "REL", 3, 1)); // 6
  // cndtions.push_back(Conditions("HUM", 0, "<", 40, "REL", 3, 0)); // 7

  // cndtions.push_back(Conditions("TMP", 1, ">", 280, "REL", 6, 1)); // 8
  // cndtions.push_back(Conditions("TMP", 1, "<", 280, "REL", 6, 0)); // 9

  // cndtions.push_back(Conditions("AMP", 2, ">", 50, "REL", 4, 1)); // 10
  // cndtions.push_back(Conditions("AMP", 2, "<", 50, "REL", 4, 0)); // 11

  // cndtions.push_back(Conditions("AMP", 2, ">", 50, "DIM", 2, 100)); // 12
  // cndtions.push_back(Conditions("AMP", 2, "<", 50, "DIM", 2, 0));   // 13

  // cndtions.push_back(Conditions("AMP", 0, "<", -5, "REL", 5, 1)); // 14
  // cndtions.push_back(Conditions("AMP", 0, ">", -5, "REL", 5, 0)); // 15

  // cndtions.push_back(Conditions("DIM", 7, ">", 100, "REL", 8, 1)); // 16
  // cndtions.push_back(Conditions("DIM", 7, "<", 100, "REL", 8, 0)); // 17

  // cndtions.push_back(Conditions("REL", 1, ">", 0, "REL", 6, 1)); // 18
  // cndtions.push_back(Conditions("REL", 1, "<", 1, "REL", 6, 0)); // 19

  uint32_t flashSize = ESP.getFlashChipSize();
  // Convert flash size from bytes to megabytes
  float flashSizeMB = (float)flashSize / (1024.0 * 1024.0);
  Serial.println("\n//======STARTING=====//");
  Serial.print("FlashSize:");
  Serial.println(flashSizeMB);
  Serial.println("Version:" + Version);
  EEPROM.begin(512);
  loadSavedValue();
  Serial.println("BLE PASS:" + String(blePass));
  bleSetPass(blePass);
  initADC();
  strip.begin();
  GyroLicense = new lisence("Gyro", "G9933");   // Key for Gyro
  VoiceLicense = new lisence("Voice", "V5612"); // Key For Voice
  // Serial.println("General Lisence:" + GeneralLisence);
  SerialBT.begin("LabobinxSmart"); // Bluetooth device name
  setupBLE();
  giveMeMacAdress();
  pinMode(34, INPUT_PULLUP); // Dimmer Protection PIN 34
  attachInterrupt(digitalPinToInterrupt(34), dimmerShortCircuitIntrupt, FALLING);
#define TasksEnabled
#ifdef TasksEnabled
  xTaskCreate(
      LOWPOWER_CONTROL_TASK,
      "LOWPOWER_CONTROL_TASK",
      3.5 * 1024, // stack size
      NULL,       // task argument
      3,          // task priority
      NULL);
  xTaskCreate(
      BLE_TASK,
      "BLE_TASK",
      3 * 1024, // stack size
      NULL,     // task argument
      1,        // task priority
      NULL);
  xTaskCreate( // I just increased it from 1.5 to 2.5 to change fome functionality and it need optimization later
      stringHandelingTask,
      "stringHandelingTask",
      3.5 * 1024, // stack size
      NULL,       // task argument
      1,          // task priority
      NULL);
  xTaskCreate(
      MainStringProcessTask,
      "MainStringProcessTask",
      5 * 1024, // stack size
      NULL,     // task argument
      3,        // task priority
      NULL);
  xTaskCreate(
      MeasurmentTask,
      "MeasurmentTask",
      4 * 1024, // stack size
      NULL,     // task argument
      2,        // task priority
      NULL);
  xTaskCreate(
      DimerTask,
      "DimerTask",
      2.5 * 1024, // stack size
      NULL,       // task argument
      4,          // task priority
      NULL);
  xTaskCreate(
      adcReadingTask,
      "ADC READING TASK",
      3 * 1024, // stack size
      NULL,     // task argument
      2,        // task priority
      NULL);
  xTaskCreate(
      led_indicator_task,
      "led_indicator_task",
      3 * 1024, // stack size
      NULL,     // task argument
      4,        // task priority
      NULL);
  xTaskCreate(
      OVR_CRNT_PRTCT_TASK,
      "OVR_CRNT_PRTCT_TASK",
      2.5 * 1024, // stack size
      NULL,       // task argument
      4,          // task priority
      NULL);
  xTaskCreate(
      I2C_SENSORS_TASK, //-------------STACK optimized up to here
      "I2C_SENSORS_TASK",
      3 * 1024, // stack size
      NULL,     // task argument
      4,        // task priority
      NULL);
  xTaskCreate(
      BatteryTask,
      "BatteryTask",
      3 * 1024,
      NULL,
      3,
      NULL);
  xTaskCreate(
      ConditionsTask,
      "ConditionsTask",
      3 * 1024,
      NULL,
      3,
      NULL);
// xTaskCreate(
//     ramMonitorTask,
//     "ramMonitorTask",
//     1024, // stack size
//     NULL, // task argument
//     1,    // task priority
//     NULL);
#endif
}
void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
void loadSavedValue()
{
  BatteryLowPrcnt = EEPROM.readFloat(E2ADD.lowVoltageSave);
  BatteryLowPrcntRelays = EEPROM.readInt(E2ADD.lowVoltageRelaysSave);
  BattLowPrcntDimers = EEPROM.readInt(E2ADD.lowVoltageDimersSave);
  BattCriticalPrcnt = EEPROM.readFloat(E2ADD.criticalVoltageSave);
  BattCriticalPrcntRelays = EEPROM.readInt(E2ADD.criticalVoltageRelaysSave);
  BattCriticalPrcntDimers = EEPROM.readInt(E2ADD.criticalVoltageDimersSave);
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
void ws2812Blink(int color)
{
#define MIN_LIGHT 4
#define MAX_LIGHT 250

  for (int i = MIN_LIGHT; i < MAX_LIGHT; i += 2)
  {
    strip.setLedColorData(0, color);
    strip.setBrightness(i);
    strip.show();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  for (int i = MAX_LIGHT; i >= MIN_LIGHT; i -= 2)
  {
    strip.setLedColorData(0, color);
    strip.setBrightness(i);
    strip.show();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void initMPU()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}
void sendAllcalibrations()
{
  char str[64];
  sprintf(str, "Vcal.val=%d\xFF\xFF\xFF", DFLT_V_CAL);
  SendToAll(str);
  sprintf(str, "Vcal.val=%d\xFF\xFF\xFF", DFLT_V_CAL);
  SendToAll(str);
  sprintf(str, "Acal.val=%d\xFF\xFF\xFF", DFLT_A_CAL);
  SendToAll(str);
  sprintf(str, "A0cal.val=%d\xFF\xFF\xFF", DFLT_A0_CAL);
  SendToAll(str);
  sprintf(str, "A2cal.val=%d\xFF\xFF\xFF", DFLT_A2_CAL);
  SendToAll(str);
  sprintf(str, "BattCap.val=%d\xFF\xFF\xFF", DFLT_BATT_CAP);
  SendToAll(str);
  sprintf(str, "Pt_mvCal.val=%d\xFF\xFF\xFF", DFLT_PT_MV_CAL);
  SendToAll(str);
  sprintf(str, "BattFullVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_FULL_VOLT);
  SendToAll(str);
  sprintf(str, "BattEmptyVolt.val=%d\xFF\xFF\xFF", DFLT_BATT_EMPTY_VOLT);
  SendToAll(str);
  sprintf(str, "BattCapTxt.val=%d\xFF\xFF\xFF", (int)batteryCap);
  SendToAll(str);
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

// END----------------------------------------------FUNCTIONS
//+++++++++++++++++++++++TO DO
//* ALI: namayeshgare gyro tooye home ham bayad scale beshe eyne too wizard
// ezafe kardane arayeyi az sw haye salem o sukhte too servis / dimerha ham
// *baraye har kodum default benevis
// dakhele loope Vcal to ya dakhele loope Tcal to infinit loop nabayad beshe
// voltage ke yehoyi biyad payin ya inke voltage eshtebah kalibre beshe rele vel mikone
// amper ke eshtebah kalibre beshe eshtebahi mire too ye protection
// *vaghti raft tooye protection mode bayad message bede ke amper kheyli ziyade va badesh ke ok shod bayad ba ye payame ok az khata darbiyad
// *password bezar baraye blt
// ezafe kardane ye delaye koochik vaghti ke calibratione chizi ro ancam dadim ta inke baes beshe tooye textbox dide beshe ta beshe baraye meghdare defaultha unu khoondesh ya mishe printesh kard too serial
// har optioni ke tooye wizard set kardi ye message bede ke set shode , toast bede
// avaz kardane hajme flash baraye inke alan flashet 2 barabar shode
// Saman for ALI: ye geraphice khoob baraye ali baraye gyro hazer kon
// connect ya disconnect shodane hame chizo mitooni tashkhis bedi az tarighe sathe voltaga
// VAGHTI GYRO RESET MISHE LOW PASS SEFR BESHE CHONKE KHEYLI TOOL MIKESHE
// M.T1 faghat hysterzis dashte bashe az tarafe ALI
// baraye 24 volt bayad devidere voltago dorost fekr koni barash chon alan majboori avazesh koni
// khazan va diod baraye filter kardane voltage esp estefade kon
// az tabeye constrain baraye limit kardane valuehat estefade kon tooye hamejaye kod ke value ha alaki naran asemoono....
//* to calibration vaghti ke baterifullcalibrate ro mizanim eshtebahi yedoone be bateryempty value ezafe mishe
//* alarmaye marboot be low power modo dorost kon
// detect floaters

// problems
//  i dont send the value from APDIM to other BLE devices in line 920