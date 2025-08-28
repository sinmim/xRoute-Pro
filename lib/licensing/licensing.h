#ifndef _LICENSING_H_
#define _LICENSING_H_

#include <Arduino.h>
#include <SPIFFS.h>
#include "obfusKeys.h"

class RegDev
{
private:
    struct regOptnsData
    {
        String name;
        String generatedKey;
        bool status;
    };

    String filePath;
    File file;
    String logContent;

    String genLis(String secretKey);
    void printStat(regOptnsData &optn);
    String readValueFromString(String str, String keyStr);
    void writeValueToString(String &str, String keyStr, String val);
    void loadLog(regOptnsData &optn);
    void savelog();

public:
    regOptnsData wrkLcns, gyroLcns, humLcns, crntLcns, gasLcns;
    RegDev(String path);
    bool openLog();
    bool isActive(regOptnsData &opt);
    bool activate(regOptnsData &optn, String key);
    void deactivate(regOptnsData &optn);
    String getKey(regOptnsData &optn)
    {
        return optn.generatedKey;
    }
};

// ===== Lizing class
class Leasing
{
private:
    String filePath;
    File file;
    String logContent;
    struct UpTime
    {
        String name;
        uint64_t value;
    };

    void loadTime(UpTime &t);
    String readValueFromString(String str, String keyStr);
    void writeValueToString(String &str, String keyStr, String val);
    void savelog();

public:
    UpTime uptime, expTime;

    Leasing(String path);
    bool openLog();
    void saveTime(UpTime &optn);
    void Uptime_tick();
    uint64_t getTime(UpTime &optn);
    void setTime(UpTime &optn, uint64_t val);
    String timeToStr(UpTime &optn)
    {
        uint64_t sec = optn.value * 600;
        uint64_t years = sec / (365 * 24 * 3600);
        sec %= (365 * 24 * 3600);                 // Remove years from total seconds
        uint64_t months = sec / (30 * 24 * 3600); // Approximate month length
        sec %= (30 * 24 * 3600);                  // Remove months from total seconds
        uint64_t days = sec / (24 * 3600);
        sec %= (24 * 3600); // Remove days from total seconds
        uint64_t hours = sec / 3600;
        sec %= 3600;                 // Remove hours from total seconds
        uint64_t minutes = sec / 60; // Remaining minutes

        return optn.name + ": " + String(years) + "Y: " + String(months) + "M: " +
               String(days) + "D: " + String(hours) + "H: " + String(minutes) + "Min";
    }
};

#endif
