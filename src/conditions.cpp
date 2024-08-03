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
void (*sendCmd)(String);
void setCmdFunction(void (*func)(String))
{
    sendCmd = func;
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
    String str = "APDIM" + String(outputPort) + ".val=" + String(val) + "\n";
    sendCmd(str.c_str());
}

void Conditions::setRel(int relNum, bool state)
{
    // it will toggle the rel state and need to set it or reset it instead of toggle . later ill do it
    String str = "sw" + String(relNum) + "\n";
    sendCmd(str.c_str());
}

void Conditions::setOut()
{
    if (outputType == "REL")
    {
        if (outputValue == 0)
        {
            setRel(outputPort, false);
        }
        else
        {
            setRel(outputPort, true);
        }
    }
    else if (outputType == "DIM")
    {
        setDim(outputValue);
    }
    Serial.println("Condition meet");
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
