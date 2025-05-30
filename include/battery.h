#ifndef _BATTERY_H
#define _BATTERY_H
//============= Defenitions
// battery types
#define BATTERY_TYPE_NON 0
#define BATTERY_TYPE_GEL 1
#define BATTERY_TYPE_ACID 2
#define BATTERY_TYPE_AGM 3
#define BATTERY_TYPE_LIFEPO4 4
#define BATTERY_TYPE_LITIUM 5
// Voltages
#define BATTERY_CONFIG_12V 1
#define BATTERY_CONFIG_24V 2
// Define a function type using a function pointer that takes a char* argument
typedef void (*FunctionType)(char *);
//======================BATTERY TABELS
const double SOC_OCV_ACID[][2] = {
    {11.62, 00.00},
    {11.63, 00.47},
    {11.67, 05.76},
    {11.71, 10.39},
    {11.75, 14.48},
    {11.79, 18.13},
    {11.83, 21.43},
    {11.87, 24.48},
    {11.91, 27.33},
    {11.95, 30.04},
    {11.99, 32.68},
    {12.03, 35.27},
    {12.07, 37.86},
    {12.11, 40.46},
    {12.15, 43.09},
    {12.19, 45.78},
    {12.23, 48.52},
    {12.27, 51.32},
    {12.31, 54.17},
    {12.35, 57.08},
    {12.39, 60.03},
    {12.43, 63.03},
    {12.47, 66.05},
    {12.51, 69.10},
    {12.55, 72.18},
    {12.59, 75.26},
    {12.63, 78.37},
    {12.67, 81.50},
    {12.71, 84.65},
    {12.75, 87.86},
    {12.79, 91.14},
    {12.83, 94.52},
    {12.87, 98.04},
    {12.89, 99.87},
    {12.89, 100.0}
    
};
const double SOC_OCV_LIFEPO4[][2] = {
    {09.99, 00.00},
    {11.30, 05.00},
    {12.11, 10.00},
    {12.58, 15.00},
    {12.85, 20.00},
    {12.99, 25.00},
    {13.06, 30.00},
    {13.10, 35.00},
    {13.12, 40.00},
    {13.13, 45.00},
    {13.14, 50.00},
    {13.16, 55.00},
    {13.17, 60.00},
    {13.18, 65.00},
    {13.20, 70.00},
    {13.21, 75.00},
    {13.22, 80.00},
    {13.25, 85.00},
    {13.30, 90.00},
    {13.40, 95.00},
    {13.70, 100.0}};//normally it is 13.6 but i changed it to be defrent with lithium
const double SOC_OCV_AGM_GEL[][2] = {
    // {10.55, 00.00},
    // {10.60, 01.00},
    // {10.70, 02.00},
    // {10.80, 03.00},
    // {10.90, 03.00},
    // {11.00, 03.00},
    // {11.10, 04.00},
    // {11.20, 06.00},
    // {11.30, 08.00},
    // {11.40, 10.00},
    // {11.50, 14.00},
    // {11.60, 19.00},
    // {11.70, 24.00},
    // {11.80, 30.00},/
    // {11.90, 37.00},
    // {12.00, 45.00},
    // {12.10, 53.00},
    // {12.20, 61.00},
    // {12.30, 69.00},
    // {12.40, 76.00},
    // {12.50, 82.00},
    // {12.60, 88.00},
    // {12.70, 92.00},
    // {12.80, 95.00},
    // {12.90, 97.00},
    // {13.00, 99.00},
    // {13.10, 100.0}
    {11.80, 00.00},
    {13.00, 100.0}
   };
const double SOC_OCV_LITIUM[][2] = {
    {10.00, 00.00},
    {11.24, 05.00},
    {12.00, 10.00},
    {12.48, 15.00},
    {12.76, 20.00},
    {12.88, 25.00},
    {12.92, 30.00},
    {12.96, 35.00},
    {12.96, 40.00},
    {13.00, 45.00},
    {13.04, 50.00},
    {13.08, 55.00},
    {13.12, 60.00},
    {13.16, 65.00},
    {13.20, 70.00},
    {13.24, 75.00},
    {13.28, 80.00},
    {13.32, 85.00},
    {13.40, 90.00},
    {13.48, 95.00},
    {13.60, 100.0}};
class Battery
{
#include <string.h>
public:
    // Battery(double angle = 0.001, double bias = 0.003, double measure = 0.03);
    void init(int type, double battery_A_H, int BatteryConfig, double v);
    void loop(double v, double a1, double B);
    double getPercent();
    void setBatType(int type);
    void setPercent(double percent);
    void setBatteryCap(double AH);
    void setBatteryArrangment(int batteryConfig);
    int getBatteryArrangment();

    double getBtPerV();
    String SelectBatteryAcordingToFullVoltage(int BatteryFullVoltage_10X, FunctionType printingFunc);
    String SelectBatteryAcordingToFullVoltage(int BatteryFullVoltage_10X);
    double getBatteryEmptyVoltage();
    double getBatteryFullVoltage();

    double batPerCoulCntr;
    double btPerV;
    int batteryType;
    double coulombCount = 0;
    double batteryCap;

private:
    unsigned long previousMillis = 0;
    unsigned long currentMillis = 0;
    double deltaTime = 0;
    double error = 0;
    double vol;
    double amp;
    double BatterArrangment;
};

#endif
