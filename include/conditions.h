#ifndef CONDITIONS_H
#define CONDITIONS_H
#include <arduino.h>
#include <string.h>
//=========
void conditionSetVariables(float *v, float *a0, float *a1, float *w, float *b, float *cwPrcnt, float *dwPrcnt, float *gwPrcnt, float *digitalTemp, float *digitalHum, float *digitalAlt, float *pt100mv, float *a2, float *battHourLeft);
//=========
void setCmdFunction(void (*func)(String));

class Conditions
{
private:
  //--Input
  int inputPort = 1;
  String inputType = "TEMP_IN";
  float *inputPtr;
  //--Oprator
  String oprt = ">";
  //--Output type
  int outputPort = 2;
  String outputType = "REL"; //"DIM"
  //--Output value
  int outputValue = 100;
  float setpoint = 75;

public:
  static int ConditionCount;
  static int getCount()
  {
    return ConditionCount;
  }
  Conditions(String inputType, int inputPort, String oprt, float setpoint, String outputType, int outputPort, int outputValue);
  void setDim(int val); // override
  void setRel(int relNum, bool state);
  void setOut();
  void doWork();
};

#endif

/*
{
  "conditions": [
    {
      "inputPort": 1,
      "inputType": "TEMP_IN",
      "operator": ">",
      "outputPort": 2,
      "outputType": "FAN",
      "outputValue": 100,
      "setpoint": 75
    },
    {
      "inputPort": 2,
      "inputType": "HUMIDITY",
      "operator": "<",
      "outputPort": 3,
      "outputType": "DEHUMIDIFIER",
      "outputValue": 50,
      "setpoint": 40
    },
    {
      "inputPort": 3,
      "inputType": "VOLTAGE",
      "operator": "=",
      "outputPort": 4,
      "outputType": "BATTERY",
      "outputValue": 12,
      "setpoint": 12
    },
    {
      "inputPort": 4,
      "inputType": "GAS",
      "operator": ">=",
      "outputPort": 5,
      "outputType": "ALARM",
      "outputValue": 1,
      "setpoint": 1
    },
    {
      "inputPort": 5,
      "inputType": "CURRENT",
      "operator": "<=",
      "outputPort": 6,
      "outputType": "CHARGER",
      "outputValue": 10,
      "setpoint": 10
    },
    {
      "inputPort": 6,
      "inputType": "SOLAR",
      "operator": "~",
      "outputPort": 7,
      "outputType": "POWER",
      "outputValue": 150,
      "setpoint": 100
    }
  ]
}
*/