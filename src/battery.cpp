#include <Arduino.h>
#include "battery.h"
#include <string.h>

double interpolate(const double table[][2], int tableSize, double x)
{
    // Check if x is out of the table bounds
    if (x < table[0][0])
    {
        return table[0][1];
    }
    if (x > table[tableSize - 1][0])
    {
        return table[tableSize - 1][1];
    }
    for (int i = 0; i < tableSize; i++)
    {
        if (x == table[i][0])
        {
            return table[i][1];
        }
    }
    int i = 0;
    while (i < (tableSize - 1))
    {
        if ((x >= table[i][0]) && (x <= table[i + 1][0]))
        {
            break;
        }
        i = i + 1;
    }
    double x1 = table[i][0];
    double x2 = table[i + 1][0];
    double y1 = table[i][1];
    double y2 = table[i + 1][1];
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}
double ACID_SOC_OCV(double v)
{
    return interpolate(SOC_OCV_ACID, sizeof(SOC_OCV_ACID) / sizeof(SOC_OCV_ACID[0]), v) / 100;
}
double LIFEPO4_SOC_OCV(double v)
{
    return interpolate(SOC_OCV_LIFEPO4, sizeof(SOC_OCV_LIFEPO4) / sizeof(SOC_OCV_LIFEPO4[0]), v) / 100;
}
double AGM_GEL_SOC_OCV(double v)
{
    return interpolate(SOC_OCV_AGM_GEL, sizeof(SOC_OCV_AGM_GEL) / sizeof(SOC_OCV_AGM_GEL[0]), v) / 100;
}
double LITIUM_SOC_OCV(double v)
{
    return interpolate(SOC_OCV_LITIUM, sizeof(SOC_OCV_LITIUM) / sizeof(SOC_OCV_LITIUM[0]), v) / 100;
}
void emptyPrint(char *str)
{
    // do noting
}
void Battery::init(int type, double battery_A_H, int BatteryConfig, double v)
{
    setBatType(type);
    setBatteryCap(battery_A_H);
    setBatteryArrangment(BatteryConfig);
    if (type == BATTERY_TYPE_AGM || type == BATTERY_TYPE_GEL)
    {
        setPercent(AGM_GEL_SOC_OCV(v / 10.0F));
    }
    else if (type == BATTERY_TYPE_ACID)
    {
        setPercent(ACID_SOC_OCV(v / 10.0F));
    }
    else if (type == BATTERY_TYPE_LIFEPO4)
    {
        setPercent(LIFEPO4_SOC_OCV(v / 10.0F));
    }
    currentMillis = millis();
    previousMillis = currentMillis;
}
/** @brief
 *  will select the battery type acording to the voltage and it also returns message if the voltage maches below values
 *  136=13.6=LITIUM_SOC_OCV
 *  129=12.9=ACID_SOC_OCV
 *  131=13.6=AGM_GEL_SOC_OCV
 *  137=13.7=LIFEPO4_SOC_OCV
 */
String Battery::SelectBatteryAcordingToFullVoltage(int BatteryFullVoltage_10X, FunctionType printingFunc)
{
    char message[64];
    BatteryFullVoltage_10X = BatteryFullVoltage_10X / BatterArrangment;
    setBatType(BATTERY_TYPE_NON);
    switch (BatteryFullVoltage_10X)
    {
    case 128: // 12.8
        setBatType(BATTERY_TYPE_ACID);
        strcpy(message, "XrouteAlarm ACID BATTERY \xFF\xFF\xFF");
        break;
    case 130: // 13.0
        setBatType(BATTERY_TYPE_AGM);
        strcpy(message, "XrouteAlarm AGM BATTERY \xFF\xFF\xFF");
        break;
    case 136: // 13.6
        setBatType(BATTERY_TYPE_LITIUM);
        strcpy(message, "XrouteAlarm LITIUM BATTERY \xFF\xFF\xFF");
        break;
    case 137: // 13.7
        setBatType(BATTERY_TYPE_LIFEPO4);
        strcpy(message, "XrouteAlarm LIFEPO4 BATTERY \xFF\xFF\xFF");
        break;
    }
    if (strlen(message))
        printingFunc(message);
    return String(message);
}
String Battery::SelectBatteryAcordingToFullVoltage(int BatteryFullVoltage_10X)
{
    return SelectBatteryAcordingToFullVoltage(BatteryFullVoltage_10X, emptyPrint);
}

/** @brief       Main Calculation loop. must be called with at least 1ms interval  */
void Battery::loop(double v, double a1, double B)
{
    currentMillis = millis();
    deltaTime = (currentMillis - previousMillis) / 1000.0F;
    previousMillis = currentMillis;
    v = v / BatterArrangment; // 24 volt system will cause deviding by 2
    vol = v / 10.0F;
    amp = a1 / 10.0F;

    if (batteryType == BATTERY_TYPE_ACID)
    {
        btPerV = ACID_SOC_OCV(vol);
    }
    else if (batteryType == BATTERY_TYPE_AGM)
    {
        btPerV = AGM_GEL_SOC_OCV(vol);
    }
    else if (batteryType == BATTERY_TYPE_LIFEPO4)
    {
        btPerV = AGM_GEL_SOC_OCV(vol);
    }
    else if (batteryType == BATTERY_TYPE_LITIUM)
    {
        btPerV = LITIUM_SOC_OCV(vol);
    }
    else if (batteryType == BATTERY_TYPE_NON)
    {
        btPerV = B;
    }

    coulombCount += (amp * deltaTime / 3600.0F) + error / 200; // Convert current from A to Ah over the time interval
    coulombCount = constrain(coulombCount, -batteryCap, batteryCap);

    batPerCoulCntr = coulombCount / batteryCap;
    error = btPerV - batPerCoulCntr;
}
/** @brief       Changing Battry Type  */
void Battery ::setBatType(int type)
{
    batteryType = type;
}
/** @brief       from 0.0F to 1.0F  */
double Battery::getPercent()
{
    return batPerCoulCntr;
}
/** @brief       from 0.0F to 1.0F for initialize the current Battery percentage for initializing Integrator */
void Battery::setPercent(double percent)
{
    coulombCount = percent * batteryCap;
};
/** @brief      Set Battery capacity in Amp Hour */
void Battery::setBatteryCap(double AH)
{
    batteryCap = AH;
};
/** @brief      from 0.0F = 0% to 1.0F = 100% read voltage dependent percent */
double Battery::getBtPerV()
{
    return btPerV;
}
void Battery::setBatteryArrangment(int batteryConfig)
{
    BatterArrangment = batteryConfig;
}
int Battery::getBatteryArrangment()
{
    return BatterArrangment;
}
double Battery::getBatteryFullVoltage()
{
    int numElements;
    double V_F;
    switch (batteryType)
    {
    case BATTERY_TYPE_GEL:
        numElements = sizeof(SOC_OCV_AGM_GEL) / sizeof(SOC_OCV_AGM_GEL[0]);
        V_F = SOC_OCV_AGM_GEL[numElements - 1][0];
    case BATTERY_TYPE_ACID:
        numElements = sizeof(SOC_OCV_ACID) / sizeof(SOC_OCV_ACID[0]);
        V_F = SOC_OCV_ACID[numElements - 1][0];
        break;
    case BATTERY_TYPE_AGM:
        numElements = sizeof(SOC_OCV_AGM_GEL) / sizeof(SOC_OCV_AGM_GEL[0]);
        V_F = SOC_OCV_AGM_GEL[numElements - 1][0];
        break;
    case BATTERY_TYPE_LIFEPO4:
        numElements = sizeof(SOC_OCV_LIFEPO4) / sizeof(SOC_OCV_LIFEPO4[0]);
        V_F = SOC_OCV_LIFEPO4[numElements - 1][0];
        break;
    case BATTERY_TYPE_LITIUM:
        numElements = sizeof(SOC_OCV_LITIUM) / sizeof(SOC_OCV_LITIUM[0]);
        V_F = SOC_OCV_LITIUM[numElements - 1][0];
        break;
    }
    return V_F;
}
double Battery::getBatteryEmptyVoltage()
{
    if (batteryType == BATTERY_TYPE_ACID)
    {
        return SOC_OCV_ACID[0][0];
    }
    else if (batteryType == BATTERY_TYPE_AGM)
    {
        return SOC_OCV_AGM_GEL[0][0];
    }
    else if (batteryType == BATTERY_TYPE_LIFEPO4)
    {
        return SOC_OCV_LIFEPO4[0][0];
    }
    else if (batteryType == BATTERY_TYPE_LITIUM)
    {
        return SOC_OCV_LITIUM[0][0];
    }
    return 0;
}