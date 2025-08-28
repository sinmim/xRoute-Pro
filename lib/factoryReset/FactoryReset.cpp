#include "FactoryReset.h"

// A struct to safely pass parameters to the FreeRTOS task.
// This avoids use-after-free issues with the FactoryReset object going out of scope.
struct ClearFlagTaskParams {
    uint32_t timeout_ms;
    std::function<void(const char *, int)> settings_set;
};

FactoryReset::FactoryReset(
    uint32_t timeout_ms,
    std::function<int(const char *, int)> settings_get,
    std::function<void(const char *, int)> settings_set)
    : _timeout_ms(timeout_ms),
      _settings_get(settings_get),
      _settings_set(settings_set) {}

int FactoryReset::check() {
    // 1. Check the reset flag from the last boot.
    int reset_flag = _settings_get(FACTORY_RESET_FLAG_KEY, 0);
    int cycle_count = 0;

    if (reset_flag == 1) {
        // The flag was set, meaning the device was reset before the timeout.
        // Increment the cycle counter.
        cycle_count = _settings_get(FACTORY_RESET_COUNTER_KEY, 0);
        cycle_count++;
        _settings_set(FACTORY_RESET_COUNTER_KEY, cycle_count);
        Serial.printf("Factory Reset: Power cycle detected. Count: %d\n", cycle_count);

    } else {
        // The flag was not set, or this is the first boot.
        // This means the last boot was clean or timed out.
        // Reset the counter.
        cycle_count = 0;
        _settings_set(FACTORY_RESET_COUNTER_KEY, 0);
        Serial.println("Factory Reset: Clean boot detected, resetting cycle count.");
    }

    // 2. Set the flag for the *next* boot.
    _settings_set(FACTORY_RESET_FLAG_KEY, 1);

    // 3. Create a parameter object on the heap to pass to the task.
    // The lambda is copied, ensuring it remains valid even after this function returns.
    ClearFlagTaskParams* params = new ClearFlagTaskParams{_timeout_ms, _settings_set};

    // 4. Create a task to clear the flag after the timeout.
    // This task will only run once and then delete itself.
    xTaskCreate(
        clearFlagTask,          // Task function
        "clearFlagTask",        // Name of the task
        2048,                   // Stack size in words
        params,                 // Task input parameter (the heap-allocated struct)
        1,                      // Priority of the task
        NULL                    // Task handle
    );

    return cycle_count;
}

void FactoryReset::clearFlagTask(void *pvParameters) {
    // Cast the parameter back to our struct pointer
    ClearFlagTaskParams *params = static_cast<ClearFlagTaskParams *>(pvParameters);

    // Wait for the specified timeout
    vTaskDelay(pdMS_TO_TICKS(params->timeout_ms));

    // After the delay, clear the reset flag and the counter using the captured lambda
    Serial.printf("Factory Reset: %ums timeout reached. Clearing flag and counter.\n", params->timeout_ms);
    params->settings_set(FACTORY_RESET_FLAG_KEY, 0);
    params->settings_set(FACTORY_RESET_COUNTER_KEY, 0);

    // Clean up the parameters struct that was allocated on the heap
    delete params;

    // The task is done, delete it to free up resources
    vTaskDelete(NULL);
}
