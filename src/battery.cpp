#include "battery.h"
#include "batterySocLookup.h"

// Helper function for linear interpolation
double interpolate(const double table[][2], int tableSize, double x)
{
    if (x <= table[0][0])
        return table[0][1];
    if (x >= table[tableSize - 1][0])
        return table[tableSize - 1][1];

    for (int i = 0; i < tableSize - 1; i++)
    {
        if (x >= table[i][0] && x <= table[i + 1][0])
        {
            double x1 = table[i][0], y1 = table[i][1];
            double x2 = table[i + 1][0], y2 = table[i + 1][1];
            return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
        }
    }
    return table[tableSize - 1][1]; // Fallback
}

Battery::Battery(BatteryType type, double capacity_Ah, BatteryConfig config, double initialVoltage_V)
{
    m_config = config;
    setBatteryCap(capacity_Ah);
    setBatType(type);

    // Set defaults for custom linear mode
    m_customEmptyVoltage_V = 11.8;
    m_customFullVoltage_V = 13.5;

    double cellVoltage = initialVoltage_V / static_cast<int>(m_config);
    updateSoCFromVoltage(cellVoltage);
    m_coulombCount_Ah = m_socFromVoltage * m_batteryCap_Ah;
    m_socFromCoulombs = m_socFromVoltage;

    m_previousMillis = millis();
    m_error = 0.0;
}

void Battery::loop(double voltage_V, double current_A)
{
    unsigned long currentMillis = millis();
    double deltaTime_s = (currentMillis - m_previousMillis) / 1000.0F;
    m_previousMillis = currentMillis;

    // Ignore unreasonably large time gaps (e.g., after a long sleep)
    if (deltaTime_s > 5.0)
        deltaTime_s = 0.0;

    double cellVoltage_V = voltage_V / static_cast<int>(m_config);

    // 1. Always get a voltage-based SoC estimate. We'll decide if we trust it.
    updateSoCFromVoltage(cellVoltage_V);

    // 2. Calculate the charge added/removed in this cycle via coulomb counting.
    double chargeAdded_Ah = (current_A * deltaTime_s) / 3600.0F;

    // 3. --- ACCOUNT FOR LOAD IMPROVEMENT ---
    // Only apply voltage-based correction when the battery is near a rest state.
    double correction_Ah = 0.0;

    if (fabs(current_A) < m_restCurrentThreshold_A)
    {
        // Battery is at rest. The voltage reading is reliable.
        m_error = m_socFromVoltage - m_socFromCoulombs;
        correction_Ah = m_error * 0.005; // Gentle correction gain
    }
    else
    {
        // Battery is under load. Do not trust voltage reading. Let coulomb count run on its own.
        m_error = 0;
    }

    // 4. Apply the coulomb count and the calculated correction.
    m_coulombCount_Ah += chargeAdded_Ah + correction_Ah;
    m_coulombCount_Ah = constrain(m_coulombCount_Ah, 0.0, m_batteryCap_Ah);
    m_socFromCoulombs = m_coulombCount_Ah / m_batteryCap_Ah;
}

void Battery::updateSoCFromVoltage(double cellVoltage_V)
{
    if (m_batteryType == BATTERY_TYPE_NON)
    {
        // Use the simple linear mode with custom voltages
        double emptyV = getEmptyVoltage();
        double fullV = getFullVoltage();
        if (fullV > emptyV)
        {
            m_socFromVoltage = (cellVoltage_V - emptyV) / (fullV - emptyV);
        }
    }
    else if (m_socOcvTable != nullptr)
    {
        // Use the chemistry-specific lookup table
        m_socFromVoltage = interpolate(m_socOcvTable, m_tableSize, cellVoltage_V) / 100.0;
    }
    m_socFromVoltage = constrain(m_socFromVoltage, 0.0, 1.0);
}

double Battery::getSoC()
{
    return m_ampSensorConnected ? m_socFromCoulombs : m_socFromVoltage;
}

void Battery::setAmpSensorConnected(bool connected)
{
    if (!m_ampSensorConnected && connected)
    {
        // When sensor is just reconnected, resync the coulomb counter with the voltage reading.
        m_coulombCount_Ah = m_socFromVoltage * m_batteryCap_Ah;
    }
    m_ampSensorConnected = connected;
}

void Battery::setBatType(BatteryType type)
{
    m_batteryType = type;
    switch (type)
    {
    case BATTERY_TYPE_ACID:
        m_socOcvTable = SOC_OCV_ACID;
        m_tableSize = sizeof(SOC_OCV_ACID) / sizeof(SOC_OCV_ACID[0]);
        Serial.println("🔋 BATTERY_TYPE_ACID");
        break;
    case BATTERY_TYPE_AGM:
    case BATTERY_TYPE_GEL:
        m_socOcvTable = SOC_OCV_AGM_GEL;
        m_tableSize = sizeof(SOC_OCV_AGM_GEL) / sizeof(SOC_OCV_AGM_GEL[0]);
        Serial.println("🔋 BATTERY_TYPE_GEL_AGM");
        break;
    case BATTERY_TYPE_LIFEPO4:
        m_socOcvTable = SOC_OCV_LIFEPO4;
        m_tableSize = sizeof(SOC_OCV_LIFEPO4) / sizeof(SOC_OCV_LIFEPO4[0]);
        Serial.println("🔋 BATTERY_TYPE_LIFEPO4");
        break;
    case BATTERY_TYPE_LITIUM:
        m_socOcvTable = SOC_OCV_LITIUM;
        m_tableSize = sizeof(SOC_OCV_LITIUM) / sizeof(SOC_OCV_LITIUM[0]);
        Serial.println("🔋 BATTERY_TYPE_LITIUM");
        break;
    default: // BATTERY_TYPE_NON
        m_socOcvTable = nullptr;
        m_tableSize = 0;
        Serial.println("🔋 BATTERY_TYPE_NON");
        break;
    }
}

void Battery::setBatteryConfig(BatteryConfig config)
{
    m_config = config;
}

void Battery::setBatteryCap(double capacity_Ah)
{
    m_batteryCap_Ah = (capacity_Ah > 0) ? capacity_Ah : 1.0;
    m_restCurrentThreshold_A = m_batteryCap_Ah / 100.0;
}

void Battery::setCustomLinearVoltages(double emptyVoltage_V, double fullVoltage_V)
{
    if (fullVoltage_V > emptyVoltage_V)
    {
        m_customEmptyVoltage_V = emptyVoltage_V;
        m_customFullVoltage_V = fullVoltage_V;
    }
    setBatType(BATTERY_TYPE_NON);
}

double Battery::getEmptyVoltage()
{
    if (m_batteryType == BATTERY_TYPE_NON)
        return m_customEmptyVoltage_V;
    if (m_socOcvTable)
        return m_socOcvTable[0][0];
    return 11.8; // Fallback
}

double Battery::getFullVoltage()
{
    if (m_batteryType == BATTERY_TYPE_NON)
        return m_customFullVoltage_V;
    if (m_socOcvTable)
        return m_socOcvTable[m_tableSize - 1][0];
    return 13.5; // Fallback
}