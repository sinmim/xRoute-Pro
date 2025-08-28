#include "SettingsStore.h"

SettingsStore::SettingsStore(const char *filePath) : _path(filePath)
{
    _mutex = xSemaphoreCreateMutex();
    _saveSignal = xSemaphoreCreateBinary(); // Use a binary semaphore for signaling
    _isDirty = false;
}

bool SettingsStore::begin()
{
    SETTINGS_LOG("Initializing SettingsStore...");
    if (!loadSettings())
    {
        SETTINGS_LOG("Warning: Failed to load settings. Will start with defaults.");
        _isDirty = true; // Mark as dirty to ensure initial defaults are saved.
    }

    BaseType_t result = xTaskCreatePinnedToCore(settingsTask, "SettingsTask", 8192, this, 1, &_taskHandle, 1);
    if (result != pdPASS)
    {
        _taskHandle = NULL;
        SETTINGS_LOG("Error: Failed to create settings task!");
        return false;
    }

    SETTINGS_LOG("Settings task started successfully.");
    return true;
}

void SettingsStore::saveIfChanged()
{
    bool needsSave = false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    needsSave = _isDirty;
    xSemaphoreGive(_mutex);

    if (needsSave)
    {
        SETTINGS_LOG("In-memory settings have changed, committing to file...");
        saveSettings();
    }
    else
    {
        SETTINGS_LOG("No settings changes to commit.");
    }
}

void SettingsStore::settingsTask(void *param)
{
    SettingsStore *self = static_cast<SettingsStore *>(param);
    while (true)
    {
        // Wait for a signal that settings have changed
        if (xSemaphoreTake(self->_saveSignal, portMAX_DELAY) == pdTRUE)
        {
            // Debounce: wait 500ms to batch any other changes that arrive close together
            vTaskDelay(pdMS_TO_TICKS(500));

            SETTINGS_LOG("Save signal received, proceeding with save.");
            self->saveIfChanged();
        }
    }
}

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
        SETTINGS_LOG("Warning: Settings file not found. Starting fresh.");
        _doc.clear();
        xSemaphoreGive(_mutex);
        return true;
    }

    File file = SPIFFS.open(_path.c_str(), "r");
    if (!file)
    {
        SETTINGS_LOG("Error: Failed to open settings file for reading.");
        xSemaphoreGive(_mutex);
        return false;
    }

    DeserializationError err = deserializeJson(_doc, file);
    file.close();

    if (err)
    {
        SETTINGS_LOG("Error: Failed to parse settings file: %s", err.c_str());
        _doc.clear();
        xSemaphoreGive(_mutex);
        return false;
    }

    _isDirty = false;
    SETTINGS_LOG("Settings loaded successfully.");
    xSemaphoreGive(_mutex);
    return true;
}

bool SettingsStore::saveSettings()
{
    if (!ensureSPIFFS())
    {
        SETTINGS_LOG("Error: SPIFFS cannot be mounted for saving.");
        return false;
    }

    // Create a temporary copy of the document to serialize.
    // This minimizes the time the mutex is locked, preventing other tasks from blocking.
    StaticJsonDocument<2048> docCopy;
    bool needsSave = false;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (_isDirty)
    {
        docCopy.set(_doc); // Deep copy the document
        _isDirty = false;  // Mark as clean, since we're about to save this state
        needsSave = true;
    }
    xSemaphoreGive(_mutex);

    if (!needsSave)
    {
        return true; // Nothing to do
    }

    SETTINGS_LOG("Saving settings to %s...", _path.c_str());
    File file = SPIFFS.open(_path.c_str(), "w");
    if (!file)
    {
        SETTINGS_LOG("Error: Failed to open settings file for writing.");
        // If save fails, re-mark as dirty so the changes aren't lost
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _isDirty = true;
        xSemaphoreGive(_mutex);
        return false;
    }

    // Serialize directly to the file stream for better performance
    if (serializeJson(docCopy, file) > 0)
    {
        SETTINGS_LOG("Settings saved successfully.");
        file.close();
        return true;
    }
    else
    {
        SETTINGS_LOG("Error: Failed to write to settings file.");
        file.close();
        // Re-mark as dirty if write fails
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _isDirty = true;
        xSemaphoreGive(_mutex);
        return false;
    }
}

bool SettingsStore::ensureSPIFFS()
{
    // The 'true' parameter formats SPIFFS if it can't be mounted
    return SPIFFS.begin(true);
}

String SettingsStore::getJson()
{
    String out;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    serializeJson(_doc, out);
    xSemaphoreGive(_mutex);
    return out;
}

String SettingsStore::getPath()
{
    return _path;
}