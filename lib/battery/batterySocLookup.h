// =========================================================================================
// FINAL RECOMMENDED STATE OF CHARGE (SoC) vs. OPEN CIRCUIT VOLTAGE (OCV) TABLES
// These tables are based on rested battery voltage at room temperature (~25°C / 77°F).
// =========================================================================================

// 1. Flooded Lead-Acid (Based on standard deep-cycle battery characteristics)
const double SOC_OCV_ACID[][2] =
    {
        {11.80, 0.0},
        {11.90, 10.0},
        {12.00, 20.0},
        {12.10, 30.0},
        {12.20, 40.0},
        {12.30, 50.0},
        {12.40, 60.0},
        {12.50, 70.0},
        {12.63, 80.0},
        {12.72, 90.0},
        {12.78, 100.0}};

// 2. LiFePO₄ (Based on the "critic's" excellent suggestion for practical usable capacity)
// 0% is defined as the rested "empty" voltage before it falls off a cliff under load.
const double SOC_OCV_LIFEPO4[][2] =
    {
        {12.80, 0.0},
        {12.90, 10.0},
        {13.00, 20.0},
        {13.08, 30.0},
        {13.14, 40.0},
        {13.18, 50.0},
        {13.21, 60.0},
        {13.24, 70.0},
        {13.28, 80.0},
        {13.32, 90.0},
        {13.35, 95.0},
        {13.40, 99.0},
        {13.50, 100.0}};

// 3. AGM / Gel (VRLA) (Corrected for rested OCV, not float voltage)
const double SOC_OCV_AGM_GEL[][2] =
    {
        {11.80, 0.0},
        {11.95, 10.0},
        {12.05, 20.0},
        {12.15, 30.0},
        {12.25, 40.0},
        {12.35, 50.0},
        {12.48, 60.0},
        {12.60, 70.0},
        {12.74, 80.0},
        {12.83, 90.0},
        {12.90, 100.0}};

// 4. Lithium-ion NMC (For a standard 3S pack, max voltage 12.6V)
const double SOC_OCV_LITIUM[][2] =
    {
        {9.30, 0.0},
        {10.50, 10.0},
        {10.80, 20.0},
        {11.01, 30.0},
        {11.19, 40.0},
        {11.34, 50.0},
        {11.49, 60.0},
        {11.67, 70.0},
        {11.88, 80.0},
        {12.15, 90.0},
        {12.60, 100.0}};