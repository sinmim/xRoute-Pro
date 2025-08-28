#ifndef ledPwm_h
#define ledPwm_h

const int LED_PWM_GPIO[] =
    {
        32,
        33,
        25,
        26,
        27,
        14,
        12};
const int channelTable[] =
    {
        0,
        2,
        6,
        14,//channel 8 flcikers
        4,
        10,
        12};

void initLED_PWM();

#endif