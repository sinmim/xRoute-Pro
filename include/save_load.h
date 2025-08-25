#ifndef SAVE_LOAD_H
#define SAVE_LOAD_H

#define FLOAT_SIZE 4 // 4 Bytes
#define EEPROM_OFFSET 0
#define ADDRESS EEPROM_OFFSET + FLOAT_SIZE
#define E2PROM_FIRST_TIME_RUN_VAL 3825 // some random numbers
#define E2PROM_NOT_FIRST_TIME_RUN_VAL 8360
#define RKO 512           // remoteKeyOfset
#define RDS (64 * 2 * 2) // remote data size:  64bit *2 edge * Size(uint16_t)

struct EEpromAdd
{
    int VcalCoSave = ADDRESS * 1;
    int ampOffsetSave = ADDRESS * 2;
    int AcalCoSave = ADDRESS * 3;
    int amp2OffsetSave = ADDRESS * 4;
    int A2calCoSave = ADDRESS * 5;
    int clnWtrMaxSave = ADDRESS * 6;
    int clnWtrMinSave = ADDRESS * 7;
    int drtWtrMaxSave = ADDRESS * 8;
    int drtWtrMinSave = ADDRESS * 9;
    int gryWtrMaxSave = ADDRESS * 10;
    int gryWtrMinSave = ADDRESS * 11;
    int batteryCapSave = ADDRESS * 12;
    int battFullVoltageSave = ADDRESS * 13;
    int battEmptyVoltageSave = ADDRESS * 14;
    int cableResistanceSave = ADDRESS * 15;
    int PT_mvCal_Save = ADDRESS * 16;
    int dimLimitSave[7] = {ADDRESS * 17, ADDRESS * 18, ADDRESS * 19, ADDRESS * 20, ADDRESS * 21, ADDRESS * 22, ADDRESS * 23};
    int NegVoltOffsetSave = ADDRESS * 24;
    int accXValueOffsetSave = ADDRESS * 25;
    int accYValueOffsetSave = ADDRESS * 26;
    int amp0OffsetSave = ADDRESS * 27;
    int A0calCoSave = ADDRESS * 28;
    int licenseStatSave = ADDRESS * 30;     // Reserve 64/4=16 Byte String for next future Variable
    int GyroOriantationSave = ADDRESS * 50; // reserve 20 Byte XY00/n
    int pressurCalOffsetSave = ADDRESS * 60;
    //int blePassSave = ADDRESS * 70;
    int E2promFirsTime = ADDRESS * 75;
    int lowVoltageSave = ADDRESS * 76;
    int lowVoltageRelaysSave = ADDRESS * 77;
    int lowVoltageDimersSave = ADDRESS * 78;
    int criticalVoltageSave = ADDRESS * 79;
    int criticalVoltageRelaysSave = ADDRESS * 80;
    int criticalVoltageDimersSave = ADDRESS * 81;

    int remoteKeysAddress[32] =
        {
            RKO + RDS * 1,
            RKO + RDS * 2,
            RKO + RDS * 3,
            RKO + RDS * 4,
            RKO + RDS * 5,
            RKO + RDS * 6,
            RKO + RDS * 7,
            RKO + RDS * 8,
            RKO + RDS * 9,
            RKO + RDS * 10,
            RKO + RDS * 12,
            RKO + RDS * 13,
            RKO + RDS * 14,
            RKO + RDS * 15,
            RKO + RDS * 16,
            RKO + RDS * 17,
            RKO + RDS * 18,
            RKO + RDS * 19,
            RKO + RDS * 20,
            RKO + RDS * 21,
            RKO + RDS * 22,
            RKO + RDS * 23,
            RKO + RDS * 24,
            RKO + RDS * 25,
            RKO + RDS * 26,
            RKO + RDS * 27,
            RKO + RDS * 28,
            RKO + RDS * 29,
            RKO + RDS * 30,
            RKO + RDS * 31,
            RKO + RDS * 32};
};

#endif
