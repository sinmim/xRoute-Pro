#ifndef CONDITIONS_H
#define CONDITIONS_H
#include <arduino.h>
#include <string.h>
//=========
void conditionSetVariables(float *v, float *a0, float *a1, float *w, float *b, float *cwPrcnt, float *dwPrcnt, float *gwPrcnt, float *digitalTemp, float *digitalHum, float *digitalAlt, float *pt100mv, float *a2, float *battHourLeft);
//=========
void setCmdFunction(void (*func)(char *str));
void getRelayStateFunction(bool (*func)(int RelNum));
void getDimValueFunction(int (*func)(int n));
class Conditions
{
private:
  int index;
  //--Input
  int inputPort;
  String inputType;
  float *inputPtr = nullptr;
  //--Oprator
  String oprt;
  //--Output type
  int outputPort;
  String outputType; //"DIM"
  //--Output value
  int outputValue;
  float setpoint;
  static bool continueRunning;

public:
  static int ConditionCount;
  
  static int getCount()
  {
    return ConditionCount;
  }
  Conditions(String inputType, int inputPort, String oprt, float setpoint, String outputType, int outputPort, int outputValue);
  ~Conditions();
  void setDim(int val); // override
  void setRel(bool state);
  void setOut();
  void doWork();
  static void stopRunning()
  {
    continueRunning=false;
  }
  static void startRunning()
  {
    continueRunning=false;
  }
  static bool isRunning()
  {
    return continueRunning;
  }
};

#endif