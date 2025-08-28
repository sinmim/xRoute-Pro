#ifndef FACTORY_RESET_H
#define FACTORY_RESET_H

#include <Arduino.h>
#include <functional>

// Key for storing the reset counter in SettingsStore
#define FACTORY_RESET_COUNTER_KEY "factory_reset_counter"
#define FACTORY_RESET_FLAG_KEY "factory_reset_flag"

class FactoryReset {
public:
    /**
     * @brief Constructor for the FactoryReset class.
     * @param timeout_ms The time in milliseconds to wait before clearing the reset flag.
     * @param settings_get A lambda function to get a value from your settings store.
     * @param settings_set A lambda function to set a value in your settings store.
     */
    FactoryReset(
        uint32_t timeout_ms,
        std::function<int(const char *, int)> settings_get,
        std::function<void(const char *, int)> settings_set);

    /**
     * @brief Call this in your setup() function to perform the check.
     * @return The number of consecutive power cycles detected.
     */
    int check();

private:
    uint32_t _timeout_ms;
    std::function<int(const char *, int)> _settings_get;
    std::function<void(const char *, int)> _settings_set;

    /**
     * @brief The task that waits for the timeout and then clears the reset flag and counter.
     * @param pvParameters Pointer to the FactoryReset instance.
     */
    static void clearFlagTask(void *pvParameters);
};

#endif // FACTORY_RESET_H
