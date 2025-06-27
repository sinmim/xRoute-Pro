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
// Comment this line out to disable all debug messages from this library
#define ENABLE_SETTINGS_STORE_DEBUG

#ifdef ENABLE_SETTINGS_STORE_DEBUG
#define SETTINGS_LOG(format, ...) printf("[settingStore] " format "\n", ##__VA_ARGS__)
#else
#define SETTINGS_LOG(format, ...)
#endif

class SettingsStore
{
public:
  String getPath()
  {
    return _path;
  }

  explicit SettingsStore(const char *filePath = "/settings.json");

  /**
   * @brief Starts the background task and loads initial settings from SPIFFS.
   * @return true on success, false on failure to create the task.
   */
  bool begin();

  /**
   * @brief Queues a key-value pair to be saved.
   * This is non-blocking. The save operation happens in a background task.
   */
  template <typename T>
  void set(const String &key, const T &value)
  {
    if (key.length() >= MAX_KEY_LEN)
    {
      SETTINGS_LOG("Error: Key is too long: %s", key.c_str());
      return;
    }
    String strValue = String(value);
    if (strValue.length() >= MAX_VAL_LEN)
    {
      SETTINGS_LOG("Error: Value is too long for key '%s'", key.c_str());
      return;
    }

    SettingsCmd cmd;
    strncpy(cmd.key, key.c_str(), MAX_KEY_LEN);
    strncpy(cmd.value, strValue.c_str(), MAX_VAL_LEN);

    if (xQueueSend(_cmdQueue, &cmd, 0) != pdPASS)
    {
      SETTINGS_LOG("Error: Settings queue is full. Failed to set key '%s'.", key.c_str());
    }
  }

  /**
   * @brief Gets a value for a given key.
   * If the key does not exist, it returns the provided default value.
   * This function is read-only and will not modify the settings file.
   */
  template <typename T>
  T get(const String &key, const T &defaultValue)
  {
    T val = defaultValue;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (_doc.containsKey(key))
    {
      JsonVariant variant = _doc[key];
      if (!variant.isNull())
      {
        val = variant.as<T>();
      }
    }
    xSemaphoreGive(_mutex);
    return val;
  }

  /**
   * @brief Gets a value for a given key.
   * Returns a default-constructed value (e.g., 0, "") if the key doesn't exist.
   */
  template <typename T>
  T get(const String &key)
  {
    return get<T>(key, T());
  }

  /**
   * @brief Loads settings from the file system. Called automatically by begin().
   * @return true on success, false on failure.
   */
  bool loadSettings();

  /**
   * @brief Returns the full settings file content as a JSON string.
   */
  String getJson();

private:
  // --- Constants for the command queue ---
  // Using fixed-size char arrays is safer for passing data between tasks
  // than using String objects, which cause memory corruption in this context.
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
  StaticJsonDocument<2048> _doc; // Using StaticJsonDocument is safer on MCUs
  SemaphoreHandle_t _mutex;
  QueueHandle_t _cmdQueue;
  TaskHandle_t _taskHandle = NULL;
};

#endif