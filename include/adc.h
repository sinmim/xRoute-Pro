#ifndef _ADC_H
#define _ADC_H

#define ADC_IN_GPIO 36//4//do not use adc2 while using wifi or blt/ble : https://github.com/khoih-prog/ESPAsync_WiFiManager
#define MUX_ADD_A 19
#define MUX_ADD_B 18
#define MUX_ADD_C 17
#define MUX_ADD_D 5

#define VOLT_MUX_IN               0
#define PT100_MUX_IN          0b1000
#define AIR_QUALITY_MUX_IN    0b0100
#define GAUGE1_MUX_IN         0b1100
#define GAUGE2_MUX_IN         0b0010
#define GAUGE3_MUX_IN         0b1010
#define AMPER_MUX_IN          0b0110
#define AMPER0_MUX_IN         0b0101
#define AIN1_MUX_IN        7
#define AIN2_MUX_IN        8
#define NEG_VOLT_MUX_IN    9
#define AIN4_MUX_IN        10
#define AIN5_MUX_IN        11
#define AIN6_MUX_IN        12
#define AIN7_MUX_IN           0b1011
#define AIN8_MUX_IN        14
#define AIN9_MUX_IN        15

void initADC(void);
void setMux(unsigned char add);
float ADC_LPF(char add, char cnt, float last, double co);
float ReadPT100_Temp(float mv,float RES_VAL);
#endif