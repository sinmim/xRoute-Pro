#ifndef CONDITIONS_H
#define CONDITIONS_H
#include <arduino.h>
#include <string.h>
// definitions
#define MOTOR_STOP 0
#define MOTOR_UP 1
#define MOTOR_DOWN 2
//=========
void conditionSetVariables(float *v, float *a0, float *a1, float *w, float *b, float *cwPrcnt, float *dwPrcnt,
                           float *gwPrcnt, float *digitalTemp, float *digitalHum, float *digitalAlt, float *pt100mv,
                           float *a2, float *battHourLeft, int *motorWay);
//=========
void setCmdFunction(void (*func)(char *str));
void getRelayStateFunction(bool (*func)(int RelNum));
void getDimValueFunction(int (*func)(int n));
class Conditions
{
private:
  String conditionType;
  // normal Coditions
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
  // Timer Conditions
  int timerPort;       // 1
  int timerUpTimeMs;   // 2000
  int timerDownTimeMs; // 1500
  int motorWay;
  int timerPortLastState = 0;
  uint32_t startTime;
  bool motorIsMovingFlg = false;

public:
  static int ConditionCount;

  static int getCount()
  {
    return ConditionCount;
  }
  Conditions(String inputType, int inputPort, String oprt, float setpoint, String outputType, int outputPort, int outputValue);
  Conditions(String _outputType, int _port, int _upTimeMs, int downTimeMs);

  ~Conditions();
  void setDim(int val); // override
  void setRel(bool state);
  void setOut();
  void stopMotor(int motorNumber);
  void doWork();
  static void stopRunning()
  {
    continueRunning = false;
  }
  static void startRunning()
  {
    continueRunning = false;
  }
  static bool isRunning()
  {
    return continueRunning;
  }
};

#endif