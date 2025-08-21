#ifndef SETTINGS_STORE_H
#define SETTINGS_STORE_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// --- Debugging ---
#define ENABLE_SETTINGS_STORE_DEBUG
#ifdef ENABLE_SETTINGS_STORE_DEBUG
#define SETTINGS_LOG(format, ...) printf("[settingStore] " format "\n", ##__VA_ARGS__)
#else
#define SETTINGS_LOG(format, ...)
#endif

class SettingsStore
{
public:
    explicit SettingsStore(const char *filePath = "/settings.json");

    bool begin();
    
    template <typename T>
    void set(const String &key, const T &value);

    template <typename T>
    T get(const String &key, const T &defaultValue);

    template <typename T>
    T get(const String &key);

    void saveIfChanged();

    bool loadSettings();
    String getJson();
    String getPath();

private:
    static const int MAX_KEY_LEN = 32;

    static void settingsTask(void *param);
    bool saveSettings();
    bool ensureSPIFFS();

    String _path;
    bool _isDirty = false;
    StaticJsonDocument<2048> _doc;
    SemaphoreHandle_t _mutex;
    SemaphoreHandle_t _saveSignal; // Replaces the queue
    TaskHandle_t _taskHandle = NULL;
};

// Implementation for the templated functions must be in the header

template <typename T>
void SettingsStore::set(const String &key, const T &value)
{
    if (key.length() >= MAX_KEY_LEN) {
        SETTINGS_LOG("Error: Key is too long: %s", key.c_str());
        return;
    }

    bool valueChanged = false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    // Only update if the key is new or the value has changed
    if (!_doc.containsKey(key) || _doc[key].as<T>() != value)
    {
        _doc[key] = value; // Let ArduinoJson handle the native type
        _isDirty = true;
        valueChanged = true;
        SETTINGS_LOG("Set '%s' in memory, marking for save.", key.c_str());
    }
    xSemaphoreGive(_mutex);
    
    // If a change was made, signal the background task to save
    if (valueChanged) {
        xSemaphoreGive(_saveSignal);
    }
}

template <typename T>
T SettingsStore::get(const String &key, const T &defaultValue)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (!_doc.containsKey(key))
    {
        SETTINGS_LOG("Key '%s' not found. Applying default value.", key.c_str());
        _doc[key] = defaultValue; // Use the native type
        _isDirty = true;
        xSemaphoreGive(_mutex);
        xSemaphoreGive(_saveSignal); // Signal task to save the new default value
        return defaultValue;
    }
    else
    {
        T val = _doc[key].as<T>();
        xSemaphoreGive(_mutex);
        return val;
    }
}

template <typename T>
T SettingsStore::get(const String &key)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    T val = _doc.containsKey(key) ? _doc[key].as<T>() : T();
    xSemaphoreGive(_mutex);
    return val;
}

#endif