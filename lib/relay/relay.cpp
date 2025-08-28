#include "relay.h"
#define DEL_NOP delayMicroseconds(100)
struct relConfig RELAYS;
bool state = REL_FREE;
bool relStates[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define SupplyVoltage 12

void setRelPWM(float volt, float SuplyVol)
{
    float duty = volt / SuplyVol;
    if (duty > 1) // if the power supply is lower than desired voltage duty must be limited to 100%
        duty = 1;
    int max = pow(2, REL_PWM_RES);
    //print all details
    //Serial.printf("volt: %.2f, SuplyVol: %.2f, duty: %.2f, max: %d, write: %d\n", volt, SuplyVol, duty, max, (int)((1 - duty) * max));
    ledcWrite(REL_PWM_CHANNEL, (1 - duty) * max); // 1-duty becaus i need inverted pwm
}
void initRelay()
{
    ledcSetup(REL_PWM_CHANNEL, REL_PWM_FREQ, REL_PWM_RES);
    ledcAttachPin(REL_PWM_PIN, REL_PWM_CHANNEL);
    setRelPWM(0, SupplyVoltage);
    pinMode(LATCH_PRT, OUTPUT);
    pinMode(CLK_PRT, OUTPUT);
    pinMode(DATA_PRT, OUTPUT);
    pinMode(RST_PRT, OUTPUT);
    setRelay(0, SupplyVoltage);
    // Serial.println("Relay init ok !");
}
void setRelayNum(int relNum, relConfig *relays, bool state)
{
    if (state == true)
    {
        relays->relPos |= (1UL << (relays->cnfgLookup[relNum - 1]));
        setRelay(relays->relPos, SupplyVoltage);
    }
    else
    {
        relays->relPos &= ~(1UL << (relays->cnfgLookup[relNum - 1]));
        setRelay(relays->relPos, SupplyVoltage);
    }
}
void setRelay(unsigned int data, float SuplyVol)
{
    state = REL_BUSY;
    //SuplyVol = SupplyVoltage; // privent  adaptive voltage for now
    int i;
    setRelPWM(REL_TRIG_VOLTAGE, SuplyVol);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    LATCH_0;
    for (i = 0; i < 16; i++)
    {
        if ((data & 0x0001) == 1)
        {
            DATA_1;
            relStates[RELAYS.cnfgLookup[i]] = true;
        }
        else
        {
            DATA_0;
            relStates[RELAYS.cnfgLookup[i]] = false;
        }
        DEL_NOP;
        CLK_1;
        DEL_NOP;
        CLK_0;
        DEL_NOP;
        data = data >> 1;
    }
    LATCH_1;
    DEL_NOP;
    RST_1;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    setRelPWM(REL_HOLD_VOLTAGE, SuplyVol);
    state = REL_FREE;
}
bool relState_0_15(int relNum)
{
    return relStates[relNum];
}
String getRelsStatStr()
{
    String str = "";
    for (int i = 0; i < 16; i++)
    {
        if (relState_0_15(i))
        {
            str += "1";
        }
        else
        {
            str += "0";
        }
    }
    return str;
}
bool relaySatat()
{
    return state;
}