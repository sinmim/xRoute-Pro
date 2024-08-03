#include<conditions.h>

//=========
float *_v, *_a0, *_a1, *_w, *_b, *_cwPrcnt, *_dwPrcnt, *_gwPrcnt, *_digitalTemp, *_digitalHum, *_digitalAlt, *_pt100mv, *_a2, *_battHourLeft;
// (10 * ReadPT100_Temp(pt100mv, 510))); // PT100
void setVariables(float *v, float *a0, float *a1, float *w, float *b, float *cwPrcnt, float *dwPrcnt, float *gwPrcnt, float *digitalTemp, float *digitalHum, float *digitalAlt, float *pt100mv, float *a2, float *battHourLeft)
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

Conditions::Conditions()
{

}

void Conditions::setDim(int val)
{
    String str = "APDIM" + String(outputPort) + ".val=" + String(val);
    sendCmd(str);
}

void Conditions::setRel(int relNum,bool state)
{
    // it will toggle the rel state and need to set it or reset it instead of toggle . later ill do it 
    String str = "sw" + String(relNum);
    sendCmd(str);
}
void Conditions::doWork()
{

}
