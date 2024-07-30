#include <Arduino.h>
#include "ledPwm.h"
#define count 7
#define RESULUTION 15
#define FREQ 1000
#define FREQ_STEP 100

void initLED_PWM()
{
    for (int i = 0; i < count; i++)
    {
        ledcSetup(channelTable[i], FREQ + FREQ_STEP * i, RESULUTION);
        ledcAttachPin(LED_PWM_GPIO[i], channelTable[i]);
    }
    Serial.println("PWM init OK !");
}
