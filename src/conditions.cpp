#include <conditions.h>
int Conditions::ConditionCount = 0;
bool Conditions::continueRunning = true;

//=========
float *_v, *_a0, *_a1, *_w, *_b, *_cwPrcnt, *_dwPrcnt, *_gwPrcnt, *_digitalTemp, *_digitalHum, *_digitalAlt, *_pt100mv, *_a2, *_battHourLeft;
int *_motorWay;
// (10 * ReadPT100_Temp(pt100mv, 510))); // PT100
void conditionSetVariables(float *v, float *a0, float *a1, float *w, float *b,
                           float *cwPrcnt, float *dwPrcnt, float *gwPrcnt,
                           float *digitalTemp, float *digitalHum, float *digitalAlt,
                           float *pt100mv, float *a2, float *battHourLeft, int *motorWay)
{
    _v = v;
    _a0 = a0;
    _a1 = a1;
    _w = w;
    _b = b;
    _cwPrcnt = cwPrcnt;
    _dwPrcnt = dwPrcnt;
    _gwPrcnt = gwPrcnt;
    _digitalTemp = digitalTemp;
    _digitalHum = digitalHum;
    _digitalAlt = digitalAlt;
    _pt100mv = pt100mv;
    _a2 = a2;
    _battHourLeft = battHourLeft;
    _motorWay = motorWay;
}
//=========

void (*sendCmd)(char *str);
void setCmdFunction(void (*func)(char *str))
{
    sendCmd = func;
};
bool (*getRel)(int RelNum);
void getRelayStateFunction(bool (*func)(int RelNum))
{
    getRel = func;
};
int (*getDim)(int DimNumber);
void getDimValueFunction(int (*func)(int n))
{
    getDim = func;
}

Conditions::Conditions(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue)
{
    conditionType = "NormalCondition";
    index = ConditionCount;
    ConditionCount++;
    inputType = _inputType;
    inputPort = _inputPort;
    oprt = _oprt;
    setpoint = _setpoint;
    outputType = _outputType;
    outputPort = _outputPort;
    outputValue = _outputValue;
    inputPtr = nullptr; // Initialize the pointer

    if (inputType == "VOLTAGE")
    {
        inputPtr = _v;
    }
    else if (inputType == "BATTERY")
    {
        inputPtr = _b;
    }
    else if (inputType == "CAR_BATTERY")
    {
    }
    else if (inputType == "CAR_BATTERY")
    {
    }
    else if (inputType == "SOLAR")
    {
    }
    else if (inputType == "CURRENT")
    {
        if (inputPort == 1)
        {
            inputPtr = _a0;
        }
        else if (inputPort == 2)
        {
            inputPtr = _a1;
        }
    }
    else if (inputType == "FLOATER")
    {
        if (inputPort == 1)
        {
            inputPtr = _cwPrcnt;
        }
        else if (inputPort == 2)
        {
            inputPtr = _gwPrcnt;
        }
        else if (inputPort == 3)
        {
            inputPtr = _dwPrcnt;
        }
    }
    else if (inputType == "TEMP_INT")
    {
        inputPtr = _digitalTemp;
    }
    else if (inputType == "TEMP_OUT")
    {
        inputPtr = _digitalTemp; // needs to change to analod pt100 value by creating a global var in the main
    }
    else if (inputType == "HUMIDITY")
    {
        inputPtr = _digitalHum;
    }
    else if (inputType == "REL")
    {
        // itll done in the do work
    }
    else if (inputType == "DIM")
    {
        // itll done in the do work
    }
    else if (inputType == "ALTITUDE")
    {
        // itll done in the do work
    }
    else if (inputType == "GAS")
    {
        inputPtr = _a2;
    }
    else if (inputType == "GYRO")
    {
    }
    else if (inputType == "POWER")
    {
    }

    Serial.println("Condition[" + String(index) + "]: IF(" + inputType + String(inputPort) + String(oprt) + String(setpoint) + ")-->" + outputType + String(outputPort) + "=" + String(outputValue));
}

Conditions::Conditions(String _outputType, int _Port, int _upTimeMs, int _downTimeMs)
{
    conditionType = "timerCondition";
    outputType = _outputType;
    index = ConditionCount;
    ConditionCount++;
    timerPort = _Port;
    outputPort = _Port;
    timerUpTimeMs = _upTimeMs;
    timerDownTimeMs = _downTimeMs;
    if (outputType == "MOTOR")
    {
        Serial.println("Condition[" + String(index) + "]: MotorUpTimer[" + String(timerUpTimeMs) + "ms]MotorDownTimer[" + String(timerDownTimeMs) + "ms]");
    }
    else if (outputType == "KEY")
    {
        Serial.println("Condition[" + String(index) + "]: " + conditionType + "(" + outputType + outputPort + ")" + "-->OnTime[" + String(timerUpTimeMs) + "ms]");
    }
}

Conditions::~Conditions()
{
    // ConditionCount--;
}

void Conditions::setDim(int val)
{
    // Serial.print("inputPort:" + String(outputPort) + "getdim:" + String(getDim(outputPort - 1)) + " val:" + String(val));
    if (getDim(outputPort - 1) != val)
    {
        char str[50];
        sprintf(str, "DIMER%d=%d\n", outputPort, val);
        sendCmd(str);
        if (conditionType == "NormalCondition")
        {
            Serial.println("Condition[" + String(index) + "]: " + inputType + String(inputPort) + oprt + String(setpoint) + "-->" + outputType + String(outputPort) + "=" + String(outputValue));
        }
    }
}

void Conditions::setRel(bool state)
{
    char str[50];
    if (getRel(outputPort - 1) != state)
    {
        if (state == true)
        {
            sprintf(str, "sw%d=ON\n", outputPort);
        }
        else
        {
            sprintf(str, "sw%d=OFF\n", outputPort);
        }
        sendCmd(str);
    }

    if (conditionType == "NormalCondition")
    {
        Serial.println("Condition[" + String(index) + "]: " + inputType + String(inputPort) + oprt + String(setpoint) + "-->" + outputType + String(outputPort) + "=" + String(outputValue));
    }
    else if (conditionType == "timerCondition")
    {
    }
}

void Conditions::stopMotor(int motorNumber)
{
    char str[16];
    sprintf(str, "Motor%d=Stop\n", motorNumber);
    sendCmd(str);
}

void Conditions::setOut()
{
    if (outputType == "REL")
    {
        if (outputValue == 0)
        {
            setRel(false);
        }
        else
        {
            setRel(true);
        }
    }
    else if (outputType == "DIM")
    {
        setDim(outputValue);
    }
}

void Conditions::doWork()
{
    if (conditionType == "NormalCondition")
    {
        float val = 1;
        if (inputType == "REL")
        {
            if (getRel(inputPort - 1) == true)
            {
                val = 1;
            }
            else
            {
                val = 0;
            }
        }
        else if (inputType == "DIM")
        {
            val = getDim(inputPort - 1);
        }
        else
        {
            val = *inputPtr;
        }

        if (oprt == ">")
        {
            if (val > setpoint)
            {
                setOut();
            }
        }
        else if (oprt == ">=")
        {
            if (val >= setpoint)
            {
                setOut();
            }
        }
        else if (oprt == "<")
        {
            if (val < setpoint)
            {
                setOut();
            }
        }
        else if (oprt == "<=")
        {
            if (val <= setpoint)
            {
                setOut();
            }
        }
        else if (oprt == "=")
        {
            if (val == setpoint)
            {
                setOut();
            }
        }
    }
    else if (conditionType == "timerCondition") // i should add a motor state reader if i want to
    {
        if (outputType == "MOTOR")
        {
            if (*_motorWay != MOTOR_STOP && !motorIsMovingFlg)
            {
                motorIsMovingFlg = true;
                startTime = millis();
            }
            if (motorIsMovingFlg)
            {
                int movingTime = millis() - startTime;
                if (*_motorWay == MOTOR_STOP)
                {
                    motorIsMovingFlg = false;
                    Serial.println("Condition[" + String(index) + "]: Finished Due to Stop");
                }

                if (*_motorWay == MOTOR_UP)
                {
                    if (movingTime > timerUpTimeMs)
                    {
                        stopMotor(timerPort);
                        Serial.println("Condition[" + String(index) + "]: IF(upTime(" + String(movingTime) + ")>" + String(timerUpTimeMs) + "ms)-->STOP");
                        motorIsMovingFlg = false;
                    }
                }
                else if (*_motorWay == MOTOR_DOWN)
                {
                    if (movingTime > timerDownTimeMs)
                    {
                        stopMotor(timerPort);
                        Serial.println("Condition[" + String(index) + "]: IF(downTime(" + String(movingTime) + ")>" + String(timerDownTimeMs) + "ms)-->STOP");
                        motorIsMovingFlg = false;
                    }
                }
            }
        }
        else if (outputType == "KEY")
        {
            if (getRel(timerPort - 1) == true && timerPortLastState == 0)
            {
                timerPortLastState = 1;
                startTime = millis();
            }
            else if (getRel(timerPort - 1) == true && timerPortLastState == 1)
            {
                uint32_t time = millis() - startTime;
                if (time >= timerUpTimeMs)
                {
                    timerPortLastState = 0;
                    setRel(false);
                    Serial.println("Condition[" + String(index) + "]: IF(OnTime(" + String(time) + ")>" + String(timerUpTimeMs) + "ms)-->OFF");
                }
            }
        }
        else if (outputType == "DIM")
        {
            if (getDim(timerPort - 1) > 0 && timerPortLastState == 0)
            {
                timerPortLastState = 1;
                startTime = millis();
            }
            else if (getDim(timerPort - 1) > 0 && timerPortLastState == 1)
            {
                uint32_t time = millis() - startTime;
                if (time >= timerUpTimeMs)
                {
                    timerPortLastState = 0;
                    setDim(0);
                    Serial.println("Condition[" + String(index) + "]: Timer(D" + outputPort + ": OnTime(" + String(time) + ")>=" + String(timerUpTimeMs) + "ms)-->OFF");
                }
            }
        }
    }
}
