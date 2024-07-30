#ifndef RELAY_H
#define RELAY_H
#include <Arduino.h>
extern int16_t relPos;
struct relConfig
{
#define REL16 7
#define REL15 6
#define REL14 5
#define REL13 4
#define REL12 3
#define REL11 2
#define REL10 1
#define REL9 0
#define REL8 15
#define REL7 14
#define REL6 13
#define REL5 12
#define REL4 11
#define REL3 10
#define REL2 9
#define REL1 8
    int cnfgLookup[16] = {REL1, REL2, REL3, REL4, REL5, REL6, REL7, REL8, REL9, REL10, REL11, REL12, REL13, REL14, REL15, REL16};
    int cnfgReverse[16] = {REL9, REL10, REL11, REL12, REL13, REL14, REL15, REL16, REL1, REL2, REL3, REL4, REL5, REL6, REL7, REL8};
    uint16_t relPos = 0;
};
#define REL_FREE 1
#define REL_BUSY 0
//-------------LATCH
#define LATCH_PIN 15
#define LATCH_PRT LATCH_PIN
#define LATCH_1 digitalWrite(LATCH_PIN, HIGH);
#define LATCH_0 digitalWrite(LATCH_PIN, LOW);
//-------------CLK
#define CLK_PIN 2
#define CLK_PRT CLK_PIN
#define CLK_1 digitalWrite(CLK_PIN, HIGH);
#define CLK_0 digitalWrite(CLK_PIN, LOW);
//-------------DATA
#define DATA_PIN 0
#define DATA_PRT DATA_PIN
#define DATA_1 digitalWrite(DATA_PIN, HIGH);
#define DATA_0 digitalWrite(DATA_PIN, LOW);
//-------------RESET
#define RST_PIN 16
#define RST_PRT RST_PIN
#define RST_1 digitalWrite(RST_PIN, HIGH);
#define RST_0 digitalWrite(RST_PIN, LOW);

//-------------PWM pin
#define REL_PWM_PIN 13
#define REL_PWM_CHANNEL 8
#define REL_PWM_RES 10
#define REL_PWM_FREQ 20000
#define REL_TRIG_VOLTAGE 12 // volt
#define REL_HOLD_VOLTAGE 7  // volt

void setRelay(unsigned int data, float SuplyVol);
void setRelayNum(int relNum, relConfig *relays, bool state);
void initRelay();
void setRelPWM(float volt, float SuplyVol);
bool relaySatat();
bool relState_0_15(int relNum);

#endif