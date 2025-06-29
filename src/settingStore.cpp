#include "SettingsStore.h"

SettingsStore::SettingsStore(const char *filePath) : _path(filePath)
{
    _mutex = xSemaphoreCreateMutex();
    _cmdQueue = xQueueCreate(10, sizeof(SettingsCmd));
    _isDirty = false;
}

bool SettingsStore::begin()
{
    SETTINGS_LOG("Initializing SettingsStore...");
    bool loadOk = loadSettings();
    if (!loadOk) {
        SETTINGS_LOG("Error: Failed to load/parse settings file. Will start with empty/default settings.");
        _isDirty = true; // Mark as dirty since we are starting fresh and will need a save
    }

    BaseType_t result = xTaskCreatePinnedToCore(settingsTask, "SettingsTask", 4096, this, 1, &_taskHandle, 1);
    if (result != pdPASS) {
        _taskHandle = NULL;
        SETTINGS_LOG("Error: Failed to create settings task!");
        return false;
    }
    
    SETTINGS_LOG("Settings task started successfully.");
    return loadOk;
}

void SettingsStore::saveIfChanged()
{
    bool needsSave = false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    needsSave = _isDirty;
    xSemaphoreGive(_mutex);

    if (needsSave) {
        SETTINGS_LOG("In-memory settings have changed, committing to file...");
        saveSettings();
    } else {
        SETTINGS_LOG("No settings changes to commit, file is up-to-date.");
    }
}

void SettingsStore::settingsTask(void *param)
{
    SettingsStore *self = static_cast<SettingsStore *>(param);
    SettingsCmd cmd;
    while (true)
    {
        if (xQueueReceive(self->_cmdQueue, &cmd, portMAX_DELAY))
        {
            SETTINGS_LOG("Received request to set '%s' = '%s'", cmd.key, cmd.value);
            self->applyAndSave(cmd.key, cmd.value);
        }
    }
}

void SettingsStore::applyAndSave(const char *key, const char *value)
{
    bool needsSave = false;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    if (!_doc.containsKey(key) || _doc[key].as<String>() != value)
    {
        _doc[key] = String(value);
        _isDirty = true; // Mark as dirty
        needsSave = true;
    }
    xSemaphoreGive(_mutex);

    if (needsSave)
    {
        saveSettings();
    }
}

bool SettingsStore::loadSettings()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    if (!ensureSPIFFS()) {
        SETTINGS_LOG("Error: SPIFFS cannot be mounted.");
        xSemaphoreGive(_mutex);
        return false;
    }

    SETTINGS_LOG("Loading settings from %s", _path.c_str());

    if (!SPIFFS.exists(_path.c_str())) {
        SETTINGS_LOG("Warning: Settings file not found. Starting fresh.");
        _doc.clear();
        xSemaphoreGive(_mutex);
        return true; // Not a failure, just a first run.
    }

    File file = SPIFFS.open(_path.c_str(), "r");
    if (!file) {
        SETTINGS_LOG("Error: Failed to open settings file for reading.");
        xSemaphoreGive(_mutex);
        return false;
    }

    DeserializationError err = deserializeJson(_doc, file);
    file.close();
    
    if (err) {
        SETTINGS_LOG("Error: Failed to parse settings file: %s", err.c_str());
        _doc.clear();
        xSemaphoreGive(_mutex);
        return false; // This IS a failure. The file is corrupt.
    }
    
    _isDirty = false; // Successfully loaded, so memory matches disk.
    SETTINGS_LOG("Settings loaded successfully.");
    //print the content of file
    String jsonStr;
    serializeJsonPretty(_doc, jsonStr);
    SETTINGS_LOG("Inside JSON: \n%s", jsonStr.c_str());
    xSemaphoreGive(_mutex);
    return true;
}

bool SettingsStore::saveSettings()
{
    if (!ensureSPIFFS()) {
        SETTINGS_LOG("Error: SPIFFS cannot be mounted for saving.");
        return false;
    }

    SETTINGS_LOG("Saving settings to %s...", _path.c_str());
    String tempJsonString;
    
    xSemaphoreTake(_mutex, portMAX_DELAY);
    serializeJson(_doc, tempJsonString);
    xSemaphoreGive(_mutex);

    File file = SPIFFS.open(_path.c_str(), "w");
    if (!file) {
        SETTINGS_LOG("Error: Failed to open settings file for writing.");
        return false;
    }

    if (file.print(tempJsonString) > 0) {
        SETTINGS_LOG("Settings saved successfully.");
        file.close();
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _isDirty = false; // Memory now matches the disk.
        xSemaphoreGive(_mutex);
        return true;
    } else {
        SETTINGS_LOG("Error: Failed to write to settings file.");
        file.close();
        return false;
    }
}

bool SettingsStore::ensureSPIFFS()
{
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