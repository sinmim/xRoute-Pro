#ifndef SETTINGS_STORE_H
#define SETTINGS_STORE_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

    /**
     * @brief Gets a value for a given key. If the key does not exist, it adds
     * the default value to the in-memory store and returns the default value.
     * Note: This may mark the settings as "dirty", requiring a save.
     */
    template <typename T>
    T get(const String &key, const T &defaultValue)
    {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (!_doc.containsKey(key))
        {
            SETTINGS_LOG("Key '%s' not found. Applying default value.", key.c_str());
            _doc[key] = String(defaultValue);
            _isDirty = true;
            xSemaphoreGive(_mutex);
            return defaultValue;
        }
        else
        {
            T val = _doc[key].as<T>();
            xSemaphoreGive(_mutex);
            return val;
        }
    }

    /**
     * @brief Gets a value for a given key. Strictly read-only.
     * Returns a default-constructed value (e.g., 0, "") if the key doesn't exist.
     */
    template <typename T>
    T get(const String &key)
    {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_doc.containsKey(key))
        {
            T val = _doc[key].as<T>();
            xSemaphoreGive(_mutex);
            return val;
        }
        xSemaphoreGive(_mutex);
        return T();
    }

    /**
     * @brief If any default values were added during 'get' calls, this
     * function saves the updated settings to the file system.
     */
    void saveIfChanged();

    bool loadSettings();
    String getJson();
    String getPath();

private:
    static const int MAX_KEY_LEN = 32;
    static const int MAX_VAL_LEN = 128;

    struct SettingsCmd
    {
        char key[MAX_KEY_LEN];
        char value[MAX_VAL_LEN];
    };

    static void settingsTask(void *param);
    void applyAndSave(const char *key, const char *value);
    bool saveSettings();
    bool ensureSPIFFS();

    String _path;
    bool _isDirty = false; // Flag to track in-memory changes
    StaticJsonDocument<2048> _doc;
    SemaphoreHandle_t _mutex;
    QueueHandle_t _cmdQueue;
    TaskHandle_t _taskHandle = NULL;
};

// Implementation for the templated 'set' function must be in the header file
template <typename T>
void SettingsStore::set(const String &key, const T &value)
{
    if (key.length() >= MAX_KEY_LEN) {
        SETTINGS_LOG("Error: Key is too long: %s", key.c_str());
        return;
    }
    String strValue = String(value);
    if (strValue.length() >= MAX_VAL_LEN) {
        SETTINGS_LOG("Error: Value is too long for key '%s'", key.c_str());
        return;
    }

    SettingsCmd cmd;
    strncpy(cmd.key, key.c_str(), MAX_KEY_LEN);
    strncpy(cmd.value, strValue.c_str(), MAX_VAL_LEN);

    if (xQueueSend(_cmdQueue, &cmd, 0) != pdPASS) {
        SETTINGS_LOG("Error: Settings queue is full. Failed to set key '%s'.", key.c_str());
    }
}

#endif