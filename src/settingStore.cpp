#include "SettingsStore.h"

// The constructor initializes the FreeRTOS synchronization primitives.
SettingsStore::SettingsStore(const char *filePath) : _path(filePath)
{
    _mutex = xSemaphoreCreateMutex();
    // Create a queue that can hold up to 10 setting change commands.
    _cmdQueue = xQueueCreate(10, sizeof(SettingsCmd));
}

// begin() starts the background task and performs the initial load.
bool SettingsStore::begin()
{
    SETTINGS_LOG("Initializing SettingsStore...");
    if (!loadSettings())
    {
        SETTINGS_LOG("Error: Failed to load initial settings. A new file will be created on the first save.");
    }

    // Create a persistent background task to handle saving settings.
    // This is more efficient than creating/destroying tasks for each save.
    // The task consumes no CPU when idle (waiting for a queue item).
    BaseType_t result = xTaskCreatePinnedToCore(
        settingsTask,   // Task function
        "SettingsTask", // Name for debugging
        4096,           // Stack size in bytes
        this,           // Parameter to pass to the task
        1,              // Priority (low)
        &_taskHandle,   // Task handle
        1               // Pin to core 1 (if on a dual-core ESP32)
    );

    if (result != pdPASS)
    {
        _taskHandle = NULL; // Ensure handle is NULL on failure
        SETTINGS_LOG("Error: Failed to create settings task!");
        return false;
    }

    SETTINGS_LOG("Settings task started successfully.");
    return true;
}

// This is the static task function that runs in the background.
void SettingsStore::settingsTask(void *param)
{
    SettingsStore *self = static_cast<SettingsStore *>(param);
    SettingsCmd cmd;

    SETTINGS_LOG("Task loop started. Waiting for settings changes.");
    while (true)
    {
        // Wait indefinitely for a command to arrive in the queue.
        if (xQueueReceive(self->_cmdQueue, &cmd, portMAX_DELAY))
        {
            SETTINGS_LOG("Received request to set '%s' = '%s'", cmd.key, cmd.value);
            self->applyAndSave(cmd.key, cmd.value);
        }
    }
}

// This function checks if the value has changed, updates the in-memory
// document, and then saves it to the file system.
void SettingsStore::applyAndSave(const char *key, const char *value)
{
    bool needsSave = false;

    xSemaphoreTake(_mutex, portMAX_DELAY);

    // Check if the key exists and if the value is different.
    if (!_doc.containsKey(key) || _doc[key].as<String>() != value)
    {
        SETTINGS_LOG("Value for '%s' changed. Updating in memory.", key);
        //_doc[key] = value; this is a bug fix for race condition
        // ***** FIX IS HERE *****
        // By creating a String object, we force ArduinoJson to make a copy
        // of the value, preventing it from being overwritten by the next command.        
        _doc[key] = String(value);
        needsSave = true;
    }
    else
    {
        SETTINGS_LOG("Value for '%s' is the same. No save needed.", key);
    }

    xSemaphoreGive(_mutex);

    // Perform the slow file I/O operation *after* releasing the mutex
    // if a change was detected. This prevents blocking other tasks that
    // might just need to quickly `get()` a value.
    if (needsSave)
    {
        saveSettings();
    }
}

// Loads the settings from the JSON file in SPIFFS into the in-memory document.
bool SettingsStore::loadSettings()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    if (!ensureSPIFFS())
    {
        SETTINGS_LOG("Error: SPIFFS cannot be mounted.");
        xSemaphoreGive(_mutex);
        return false;
    }

    SETTINGS_LOG("Loading settings from %s", _path.c_str());

    if (!SPIFFS.exists(_path.c_str()))
    {
        SETTINGS_LOG("Warning: Settings file not found. Starting with empty settings.");
        _doc.clear();
        xSemaphoreGive(_mutex);
        return true; // Not an error, just means we start fresh.
    }

    File file = SPIFFS.open(_path.c_str(), "r");
    if (!file)
    {
        SETTINGS_LOG("Error: Failed to open settings file for reading.");
        xSemaphoreGive(_mutex);
        return false;
    }

    // Deserialize the JSON document from the file.
    DeserializationError err = deserializeJson(_doc, file);
    file.close();

    if (err)
    {
        SETTINGS_LOG("Error: Failed to parse settings file: %s", err.c_str());
        _doc.clear(); // Clear potentially corrupt data
        xSemaphoreGive(_mutex);
        return false;
    }

    SETTINGS_LOG("Settings loaded successfully.");
    xSemaphoreGive(_mutex);
    return true;
}

// Saves the in-memory JSON document to the file in SPIFFS.
bool SettingsStore::saveSettings()
{
    if (!ensureSPIFFS())
    {
        SETTINGS_LOG("Error: SPIFFS cannot be mounted for saving.");
        return false;
    }

    SETTINGS_LOG("Saving settings to %s...", _path.c_str());

    // Create a temporary string to hold the serialized data.
    // This avoids holding the mutex during the slow file write operation.
    String tempJsonString;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    serializeJson(_doc, tempJsonString);
    xSemaphoreGive(_mutex);

    File file = SPIFFS.open(_path.c_str(), "w");
    if (!file)
    {
        SETTINGS_LOG("Error: Failed to open settings file for writing.");
        return false;
    }

    // Write the serialized string to the file.
    if (file.print(tempJsonString) > 0)
    {
        SETTINGS_LOG("Settings saved successfully.");
        file.close();
        return true;
    }
    else
    {
        SETTINGS_LOG("Error: Failed to write to settings file.");
        file.close();
        return false;
    }
}

// Helper to ensure SPIFFS is started.
bool SettingsStore::ensureSPIFFS()
{
    // The `true` parameter formats SPIFFS if it fails to mount.
    if (!SPIFFS.begin(true))
    {
        return false;
    }
    return true;
}

// Returns the current settings as a JSON-formatted string.
String SettingsStore::getJson()
{
    String out;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    serializeJson(_doc, out);
    xSemaphoreGive(_mutex);
    return out;
}