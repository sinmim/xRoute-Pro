// SettingsStore.cpp
#include "SettingsStore.h"

SettingsStore::SettingsStore(const char *filePath)
    : _path(filePath), _changed(false)
{
    // No auto-load; call loadUserData() explicitly.
}

bool SettingsStore::ensureSPIFFS()
{
    // Pass 'true' to format SPIFFS if mounting fails.
    if (!SPIFFS.begin(true))
    {
        Serial.println("ERROR: SPIFFS mount failed!");
        return false;
    }
    return true;
}

bool SettingsStore::loadUserData()
{
    if (!ensureSPIFFS())
    {
        return false;
    }

    // If file doesn't exist, start with an empty document.
    if (!SPIFFS.exists(_path))
    {
        _doc.clear();
        _changed = false;
        return true;
    }

    File file = SPIFFS.open(_path, FILE_READ);
    if (!file)
    {
        Serial.print("ERROR: Cannot open ");
        Serial.print(_path);
        Serial.println(" for reading.");
        return false;
    }

    DeserializationError err = deserializeJson(_doc, file);
    file.close();
    if (err)
    {
        Serial.print("ERROR: Failed to parse JSON at ");
        Serial.print(_path);
        Serial.print(": ");
        Serial.println(err.c_str());
        _doc.clear();
        _changed = false;
        return false;
    }

    // Successfully loaded—no pending changes.
    _changed = false;
    return true;
}

bool SettingsStore::saveUserData()
{
    if (!ensureSPIFFS())
    {
        return false;
    }

    File file = SPIFFS.open(_path, FILE_WRITE);
    if (!file)
    {
        Serial.print("ERROR: Cannot open ");
        Serial.print(_path);
        Serial.println(" for writing.");
        return false;
    }

    // Serialize as JSON (human-readable). Use serializeJson(...) if you want less overhead.
    if (serializeJson(_doc, file) == 0)
    {
        Serial.print("ERROR: No data written to ");
        Serial.println(_path);
        file.close();
        return false;
    }
    file.close();

    _changed = false;
    return true;
}

bool SettingsStore::waitForChange(uint32_t timeoutMs)
{
    uint32_t start = millis();
    while (!_changed)
    {
        if (millis() - start >= timeoutMs)
        {
            return false; // timed out
        }
        delay(10);
    }
    return true; // a set()/get-insert occurred
}

String SettingsStore::getPath()
{
    return _path;
}

String SettingsStore::getJson()
{
    String str;
    serializeJson(_doc, str);
    return str;
}