#include <Arduino.h>
#include "adc.h"
#include "dsp.h"
#define PT100_MUX_IN 0b1000
int MUX_GPIO[] = {MUX_ADD_D, MUX_ADD_C, MUX_ADD_B, MUX_ADD_A};
/*Mode: reversed (x,y = y,x) analysis
Polynomial degree 12, 16 x,y data pairs.
Correlation coefficient = 0.9999986111605647
Standard error = 0.7298184549825023

Output form: C function:
*/
// I bypassed the regresion due to liniear behavor of adc after making the pin 4 input
double regress(double x)
{
    return x;
    double terms[] = {
        -1.6766759377111345e+001,
        1.5932397689935209e+000,
        2.6298001083367077e-004,
        -5.7889720753085387e-007,
        3.8930327022030423e-010,
        -1.2907445983030062e-013,
        1.6615272468837591e-017};

    size_t csz = sizeof terms / sizeof *terms;

    double t = 1;
    double r = 0;
    for (int i = 0; i < csz; i++)
    {
        r += terms[i] * t;
        t *= x;
    }
    return r;
}
// Copyright (c) 2019, P. Lutus -- http://arachnoid.com. All Rights Reserved.
void initADC(void)
{
    pinMode(4, INPUT);
    pinMode(36, INPUT);

    for (int i = 0; i < 4; i++)
    {
        pinMode(MUX_GPIO[i], OUTPUT);
    }
    analogSetWidth(12); // 12bit
    //Serial.println("ADC MUX init ok !");
    if (false)
    {
        float refVol = 0;
        setMux(VOLT_MUX_IN);
        Serial.println("REF VOL(mv) , adc (mv)");
        while (true) // for regresion calibration you need to active this
        {
            static float v;
            // v = LOW_PASS_FILTER(regress((analogReadMilliVolts(ADC_IN_GPIO))), v, 0.998);

            Serial.print(refVol += 500);
            Serial.print(" , ");
            while (Serial.available() == 0)
            {
                v = LOW_PASS_FILTER((analogReadMilliVolts(ADC_IN_GPIO)), v, 0.998);
            };

            while (Serial.available())
            {
                Serial.read();
            }

            for (int i = 0; i < 1000; i++)
            {
                v = LOW_PASS_FILTER((analogReadMilliVolts(ADC_IN_GPIO)), v, 0.998);
            }

            // v = LOW_PASS_FILTER((analogRead(ADC_IN_GPIO)), v, 0.998);
            Serial.println(v);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}
void setMux(unsigned char add)
{
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(MUX_GPIO[i], ((add & 0b0001) == 1) ? HIGH : LOW);
        add >>= 1;
        add &= 0b1111;
    }
}
float ADC_LPF(char add, char cnt, float last, double co)
{
    // static bool adcBussyFlg = false;
    static char lastAdd = 0;

    if (add != lastAdd)
    {
        lastAdd = add;
        setMux(add);
        delay(1);
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    while (cnt--)
    {

        double tmp = regress(analogReadMilliVolts(ADC_IN_GPIO));
        if ((abs(tmp - last) / last) < 0.2)
        {
            // co = 1 - abs(last - tmp) / fmax(last, tmp);
            last = LOW_PASS_FILTER(tmp, last, co);
        }
        else
        {
            last = LOW_PASS_FILTER(regress(analogReadMilliVolts(ADC_IN_GPIO)), last, co);
        }
    }
    return last;
}
float ReadPT100_Temp(float mv, float RES_VAL)
{
    float ptemp;
    float res = (mv / (3324 - mv)) * RES_VAL; // 1750;

    ptemp = (res * res * 0.001189208) + (res * 2.305943) - 242.3386;
    return ptemp;
}