#include "licensing.h"
#include <SHA256.h>
#include "obfusKeys.h"

// ===================== RegDev Implementation =====================
RegDev::RegDev(String path)
{
    filePath = path;
    wrkLcns.name = "Working License";
    gyroLcns.name = "Gyro License";
    humLcns.name = "Humidity License";
    crntLcns.name = "Current License";
    gasLcns.name = "Gas License";
    // wrkLcns.generatedKey = genLis(wrkLcnsScrtKey);
    // gyroLcns.generatedKey = genLis(gyroLcnsScrtKey);
    // humLcns.generatedKey = genLis(humLcnsScrtKey);
    // crntLcns.generatedKey = genLis(crntLcnsScrtKey);
    // gasLcns.generatedKey = genLis(gasLcnsScrtKey);
    wrkLcns.generatedKey = genLis(deobfuscate_rolling(obfWorKey).c_str());
    gyroLcns.generatedKey = genLis(deobfuscate_rolling(obfGyrKey).c_str());
    humLcns.generatedKey = genLis(deobfuscate_rolling(obfHumKey).c_str());
    crntLcns.generatedKey = genLis(deobfuscate_rolling(obfCurKey).c_str());
    gasLcns.generatedKey = genLis(deobfuscate_rolling(obfGasKey).c_str());
}

String RegDev::genLis(String secretKey)
{
    uint64_t uid = ESP.getEfuseMac();
    char seed[32];
    snprintf(seed, sizeof(seed), "%012llX%s", uid, secretKey.c_str());

    SHA256 sha256;
    uint8_t hash[32];
    sha256.update((const uint8_t *)seed, strlen(seed));
    sha256.finalize(hash, sizeof(hash));

    char realSerial[18];
    snprintf(realSerial, 17, "%02X%02X%02X%02X%02X%02X%02X%02X",
             hash[0], hash[1], hash[2], hash[3], hash[4], hash[5], hash[6], hash[7]);
    return String(realSerial);
}

bool RegDev::openLog()
{
    file = SPIFFS.open(filePath, FILE_READ);
    if (!file)
    {
        Serial.println("Error: Could not open file for reading!");
        return false;
    }

    logContent = file.readString();
    file.close(); // Close file after reading

    if (logContent.isEmpty())
    {
        logContent = wrkLcns.name + "=0123456789ABCDEF\n" +
                     gyroLcns.name + "=52CAD7890B66749D\n" +
                     humLcns.name + "=0123456789ABCDEF\n" +
                     crntLcns.name + "=0123456789ABCDEF\n";

        Serial.println("Creating license log file for first time!");
        savelog(); // Save default content
    }
    else
    {
        loadLog(wrkLcns);
        loadLog(gyroLcns);
        loadLog(humLcns);
        loadLog(crntLcns);
        loadLog(gasLcns);
        Serial.println("Licenses =======> WORK:" + String(wrkLcns.status) + " GYRO:" + String(gyroLcns.status) +
                       +" HUM:" + String(humLcns.status) + " CUR:" + String(crntLcns.status) + " GAS:" + String(gasLcns.status));
    }

    return true;
}

void RegDev::printStat(regOptnsData &optn)
{
    Serial.println(optn.name + ":" + String(optn.status));
}

String RegDev::readValueFromString(String str, String keyStr)
{
    int sIndex = str.indexOf(keyStr) + keyStr.length() + 1; // +1 for '='
    int eIndex = str.indexOf("\n", sIndex);
    return str.substring(sIndex, eIndex);
}

void RegDev::writeValueToString(String &str, String keyStr, String val)
{
    int sIndex = str.indexOf(keyStr + "="); // Find "keyStr="
    if (sIndex == -1)
    {
        str += keyStr + "=" + val + '\n';
        Serial.println("NEW VALL ADDED : " + str + "\nEND");
        return;
    }

    sIndex += keyStr.length() + 1; // Move to the start of the value

    int eIndex = str.indexOf("\n", sIndex); // Find the end of the value (next newline)
    if (eIndex == -1)
        eIndex = str.length(); // If no newline, go to end of string

    // Reconstruct string: keep part before sIndex, add new value, keep part after eIndex
    str = str.substring(0, sIndex) + val + str.substring(eIndex);
}

void RegDev::loadLog(regOptnsData &optn)
{
    String tmp = readValueFromString(logContent, optn.name);
    optn.status = (tmp == optn.generatedKey);
}

void RegDev::savelog()
{
    // Open file in write mode
    file = SPIFFS.open(filePath, FILE_WRITE);
    if (!file)
    {
        Serial.println("Error: Could not open file for writing!");
        return;
    }

    // Write content to file
    file.print(logContent);
    file.close(); // Always close file after writing

    // Debugging: Reopen file to verify written content
    file = SPIFFS.open(filePath, FILE_READ);
    if (file)
    {
        Serial.println("Saved Log Content:");
        Serial.println(file.readString()); // Read and print saved content
        file.close();
    }
}

bool RegDev::isActive(regOptnsData &opt)
{
    return opt.status;
}

bool RegDev::activate(regOptnsData &optn, String key)
{
    if (key == optn.generatedKey)
    {
        optn.status = true;
        // Serial.println("KEY IS OK!");
        writeValueToString(logContent, optn.name, key);
        savelog();
        return true;
    }
    Serial.println("KEY IS WRONG!" + String(optn.generatedKey) + "~" + String(key));
    printStat(optn);
    return false;
}

void RegDev::deactivate(regOptnsData &optn)
{
    optn.status = false;
    writeValueToString(logContent, optn.name, "0000000000000000");
    savelog();
    printStat(optn);
}

// ===================== Leasing Implementation =====================
Leasing::Leasing(String path)
{
    filePath = path;
    uptime.name = "Uptime";
    expTime.name = "ExpireTime";
}

bool Leasing::openLog()
{
    // Open file in READ mode first
    file = SPIFFS.open(filePath, FILE_READ);
    if (!file)
    {
        Serial.println("Error: Could not open Leasing file for reading! Creating new file...");

        // If file does not exist, create it in WRITE mode
        file = SPIFFS.open(filePath, FILE_WRITE);
        if (!file)
        {
            Serial.println("Error: Could not create new Leasing file!");
            return false;
        }

        logContent = uptime.name + "=0\n";
        file.print(logContent);
        file.close();
        Serial.println("Created new Leasing log file.");
        return true;
    }

    // Read existing content
    logContent = file.readString();

    file.close(); // Close file after reading

    if (logContent.isEmpty())
    {
        Serial.println("Leasing log file is empty! InitiaLeasing...");
        logContent = uptime.name + "=0\n" +
                     expTime.name + "=0\n";
        savelog();
    }
    else
    {
        loadTime(uptime);
        loadTime(expTime);
        // printing log content
        Serial.println("//*****************inside logContent");
        Serial.println(logContent);
        Serial.println("//*****************values");
        Serial.println(uptime.value);
        Serial.println(expTime.value);
        Serial.println("//*****************values");

        Serial.println("Leasing Timer: " + String(uptime.value));
    }

    return true;
}

void Leasing::loadTime(UpTime &optn)
{
    String str = readValueFromString(logContent, optn.name);
    optn.value = str.toInt();
}

String Leasing::readValueFromString(String str, String keyStr)
{
    int sIndex = str.indexOf(keyStr) + keyStr.length() + 1; // +1 for '='
    int eIndex = str.indexOf("\n", sIndex);
    return str.substring(sIndex, eIndex);
}

void Leasing::writeValueToString(String &str, String keyStr, String val)
{
    int sIndex = str.indexOf(keyStr + "="); // Find "keyStr="
    if (sIndex == -1)
    {
        str += keyStr + "=" + val + '\n';
        Serial.println("NEW VALL ADDED : " + str + "\nEND");
        return;
    }
    sIndex += keyStr.length() + 1; // Move to the start of the value

    int eIndex = str.indexOf("\n", sIndex); // Find the end of the value (next newline)
    if (eIndex == -1)
        eIndex = str.length(); // If no newline, go to end of string

    // Reconstruct string: keep part before sIndex, add new value, keep part after eIndex
    str = str.substring(0, sIndex) + val + str.substring(eIndex);
}

void Leasing::savelog()
{
    // Open file in write mode
    file = SPIFFS.open(filePath, FILE_WRITE);
    if (!file)
    {
        Serial.println("Error: Could not open file for writing!");
        return;
    }
    // Write content to file
    file.print(logContent);
    file.close(); // Always close file after writing
    // Debugging: Reopen file to verify written content
    file = SPIFFS.open(filePath, FILE_READ);
    if (file)
    {
        Serial.println("Saved Log Content:");
        Serial.println(file.readString()); // Read and print saved content
        file.close();
    }
}

void Leasing::saveTime(UpTime &optn)
{
    writeValueToString(logContent, optn.name, String(optn.value));
    savelog();
}

void Leasing::Uptime_tick()
{
    uptime.value++;
}

uint64_t Leasing::getTime(UpTime &optn)
{
    return optn.value;
}

void Leasing::setTime(UpTime &optn, uint64_t val)
{
    optn.value = val;
    saveTime(optn);
}
