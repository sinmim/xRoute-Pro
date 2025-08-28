#ifndef _BATTERY_H
#define _BATTERY_H

#include <Arduino.h>

// Battery chemistry types
enum BatteryType
{
    BATTERY_TYPE_NON = 0, // Used for custom linear voltage mode
    BATTERY_TYPE_GEL = 1,
    BATTERY_TYPE_ACID = 2,
    BATTERY_TYPE_AGM = 3,
    BATTERY_TYPE_LIFEPO4 = 4,
    BATTERY_TYPE_LITIUM = 5
};
// create a string enum of battery names
const char *const BATTERY_TYPE_NAMES[] =
    {
        "NON",
        "GEL",
        "ACID",
        "AGM",
        "LIFEPO4",
        "LITIUM"};

// Battery system voltage configuration (for series connections)
enum class BatteryConfig
{
    V12 = 1,
    V24 = 2
};

// Extern declarations for the lookup tables defined in battery.cpp
extern const double SOC_OCV_ACID[][2];
extern const double SOC_OCV_LIFEPO4[][2];
extern const double SOC_OCV_AGM_GEL[][2];
extern const double SOC_OCV_LITIUM[][2];

class Battery
{
public:
    /**
     * @brief Construct a new Battery object.
     * @param type The type of the battery (e.g., BATTERY_TYPE_AGM).
     * @param capacity_Ah The total capacity of the battery in Ampere-hours.
     * @param config The system voltage configuration (12V or 24V).
     * @param initialVoltage_V The current battery voltage in Volts for initial SoC estimation.
     */
    Battery(BatteryType type, double capacity_Ah, BatteryConfig config, double initialVoltage_V);

    /**
     * @brief Main calculation loop. Must be called periodically (e.g., every 100ms).
     * @param voltage_V The raw measured system voltage in Volts.
     * @param current_A The raw measured system current in Amps (positive for charging).
     */
    void loop(double voltage_V, double current_A);

    /**
     * @brief Gets the best available State of Charge (SoC) as a fraction (0.0 to 1.0).
     * @return Coulomb-counted SoC if amp sensor is connected, otherwise falls back to voltage-based SoC.
     */
    double getSoC();

    /**
     * @brief Informs the class about the status of the external current sensor.
     * @param connected True if the sensor is connected and providing valid readings.
     */
    void setAmpSensorConnected(bool connected);

    /**
     * @brief Changes the battery chemistry profile at runtime.
     * @param type The new battery type from the BatteryType enum.
     */
    void setBatType(BatteryType type);

    String getBatTypeName();

    /**
     * @brief Updates the battery system voltage configuration (12V/24V) at runtime.
     * @param config The new system voltage configuration (V12 or V24).
     */
    void setBatteryConfig(BatteryConfig config);

    /**
     * @brief Updates the battery's total capacity at runtime.
     * @param capacity_Ah New capacity in Ampere-hours.
     */
    void setBatteryCap(double capacity_Ah);

    /**
     * @brief Configures the battery to use simple linear interpolation for SoC.
     * @param emptyVoltage_V The voltage the user considers to be 0%.
     * @param fullVoltage_V The voltage the user considers to be 100%.
     */
    void setCustomLinearVoltages(double emptyVoltage_V, double fullVoltage_V);

private:
    double m_restCurrentThreshold_A;
    // Configuration
    BatteryType m_batteryType;
    double m_batteryCap_Ah;
    BatteryConfig m_config;
    bool m_ampSensorConnected = false;

    // State variables
    double m_coulombCount_Ah;
    double m_socFromVoltage;
    double m_socFromCoulombs;
    double m_error;

    // Timing
    unsigned long m_previousMillis;

    // Lookup Table Pointers
    const double (*m_socOcvTable)[2];
    int m_tableSize;

    // Custom linear mode variables
    double m_customEmptyVoltage_V;
    double m_customFullVoltage_V;

    // Private Helper Methods
    void updateSoCFromVoltage(double cellVoltage_V);
    double getEmptyVoltage();
    double getFullVoltage();
};

#endif