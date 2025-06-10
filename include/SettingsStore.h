// SettingsStore.h
#ifndef SETTINGS_STORE_H
#define SETTINGS_STORE_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

/*
  SettingsStore

  - Stores arbitrary key/value pairs in a JSON file on SPIFFS.
  - Constructor takes a SPIFFS file path (e.g. "/config/wifi.json").
  - get<T>(key, defaultValue):
      • If 'key' exists, returns its value as T.
      • If missing, inserts defaultValue, immediately saves JSON, and returns defaultValue.
  - set<T>(key, value):
      • If new value ≠ existing, updates JSON[key] = value, immediately saves.
      • If identical, does nothing.
  - loadUserData():     Loads JSON from the given path (or starts empty if missing).
  - saveUserData():     Writes in-memory JSON to SPIFFS.
  - waitForChange(ms):  Blocks until a set/get-insert occurs or timeout.
*/

class SettingsStore
{
public:
  // Pass your desired SPIFFS path, e.g. "/config/settings.json".
  // Defaults to "/settings.json" if omitted.
  explicit SettingsStore(const char *filePath = "/settings.json");

  // Load whatever JSON is saved at _path (or start empty if none).
  // Returns true on success.
  bool loadUserData();

  // Write current in-memory JSON to SPIFFS. Returns true on success.
  bool saveUserData();

  // Generic getter:
  //   - If 'key' exists, returns _doc[key].as<T>().
  //   - Otherwise, inserts initializer, saves immediately, and returns initializer.
  template <typename T>
  T get(const String &key, const T &initializer)
  {
    if (_doc.containsKey(key))
    {
      return _doc[key].as<T>();
    }
    _doc[key] = initializer;
    saveUserData();
    return initializer;
  }
  // get without initializer
  template <typename T>
  T get(const String &key)
  {
    if (_doc.containsKey(key))
    {
      return _doc[key].as<T>();
    }
    return T(); // Return default-constructed value for the type T
  }

  // Generic setter:
  //   - If JSON[key] == value, does nothing.
  //   - Otherwise, updates JSON[key] = value and immediately saveUserData().
  template <typename T>
  void set(const String &key, const T &value)
  {
    if (_doc.containsKey(key))
    {
      if (_doc[key].as<T>() == value)
      {
        return;
      }
    }
    _doc[key] = value;
    saveUserData();
  }

  // Overload for C‐string values (stored as Arduino String):
  void set(const String &key, const char *value)
  {
    if (_doc.containsKey(key))
    {
      String old = _doc[key].as<String>();
      if (old.equals(value))
      {
        return;
      }
    }
    _doc[key] = String(value);
    saveUserData();
  }

  // If you want to wait until any set/get-insert happens:
  // Returns true if a change occurred before timeoutMs, false if timed out.
  bool waitForChange(uint32_t timeoutMs);

  // some more helpers by saman
  String getPath();
  String getJson();

private:
  String _path;                   // SPIFFS path for this instance's JSON
  bool _changed;                  // Becomes true on set()/get-insert(); reset after save
  DynamicJsonDocument _doc{1024}; // Adjust size if you need more fields

  // Mounts (or formats) SPIFFS if necessary.
  bool ensureSPIFFS();
};

#endif // SETTINGS_STORE_H
