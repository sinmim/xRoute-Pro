#include <conditions.h>
int Conditions::ConditionCount = 0;
//=========
float *_v, *_a0, *_a1, *_w, *_b, *_cwPrcnt, *_dwPrcnt, *_gwPrcnt, *_digitalTemp, *_digitalHum, *_digitalAlt, *_pt100mv, *_a2, *_battHourLeft;
// (10 * ReadPT100_Temp(pt100mv, 510))); // PT100
void conditionSetVariables(float *v, float *a0, float *a1, float *w, float *b, float *cwPrcnt, float *dwPrcnt, float *gwPrcnt, float *digitalTemp, float *digitalHum, float *digitalAlt, float *pt100mv, float *a2, float *battHourLeft)
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

Conditions::Conditions(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue)
{
    ConditionCount++;

    inputType = _inputType;
    inputPort = _inputPort;
    oprt = _oprt;
    setpoint = _setpoint;
    outputType = _outputType;
    outputPort = _outputPort;
    outputValue = _outputValue;
    inputPtr = nullptr; // Initialize the pointer

    if (inputType == "VOL")
    {
        inputPtr = _v;
    }
    else if (inputType == "BAT")
    {
        inputPtr = _b;
    }
    else if (inputType == "AMP")
    {
        if (inputPort == 0)
        {
            inputPtr = _a0;
        }
        else if (inputPort == 1)
        {
            inputPtr = _a1;
        }
        else if (inputPort == 2)
        {
            inputPtr = _a2;
        }
    }
    else if (inputType == "FLT")
    {
        if (inputPort == 0)
        {
            inputPtr = _cwPrcnt;
        }
        else if (inputPort == 1)
        {
            inputPtr = _gwPrcnt;
        }
        else if (inputPort == 2)
        {
            inputPtr = _dwPrcnt;
        }
    }
    else if (inputType == "TMP")
    {
        if (inputPort == 0)
        {
            inputPtr = _digitalTemp; // needs to change to analod pt100 value by creating a global var in the main
        }
        else if (inputPort == 1)
        {
            inputPtr = _digitalTemp;
        }
    }
    else if (inputType == "HUM")
    {
        inputPtr = _digitalHum;
    }
    else if (inputType == "REL")
    {
        // inputPtr = ; must add a variable for relay
    }
}

void Conditions::setDim(int val)
{
    char str[50];
    sprintf(str, "APDIM%d.val=%d\n", outputPort, val);
    sendCmd(str);
}

void Conditions::setRel(bool state)
{
    char str[50];
    if (getRel(outputPort-1) != state)
    {
        sprintf(str, "sw%d\n", outputPort);
        sendCmd(str);
    }
}

void Conditions::setOut()
{
    if (outputType == "REL")
    {
        if (outputValue == 0)
        {
            setRel( false);
        }
        else
        {
            setRel( true);
        }
    }
    else if (outputType == "DIM")
    {
        setDim(outputValue);
    }
}
void Conditions::doWork()
{
    float val = *inputPtr;

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
    else if (oprt == "==")
    {
        if (val == setpoint)
        {
            setOut();
        }
    }
}
