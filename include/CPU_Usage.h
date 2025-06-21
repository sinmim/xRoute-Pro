#ifndef CPU_USAGE_H
#define CPU_USAGE_H

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define the number of CPU cores based on the ESP32 configuration
#if CONFIG_FREERTOS_UNICORE
#define CPU_CORE_COUNT 1
#else
#define CPU_CORE_COUNT 2
#endif

class CPU_Usage {
public:
    CPU_Usage();
    void begin(uint32_t calibration_duration_ms = 2000, uint32_t update_interval_ms = 1000);
    float get_cpu_usage_core0();
#if CPU_CORE_COUNT > 1
    float get_cpu_usage_core1();
    float get_cpu_usage_total();
#endif

private:
    TaskHandle_t _monitoring_task_handle;
    static volatile uint32_t _idle_task_counter_core0;
    static volatile uint32_t _idle_task_counter_core1;

    // --- FIX: Changed return type from void to bool ---
    static bool idle_hook_core0();
    static bool idle_hook_core1();

    void monitoring_task_body();
    static void monitoring_task_wrapper(void* _this);
    void update_cpu_stats();
    
    float _baseline_idle_count_per_sec_core0;
    float _baseline_idle_count_per_sec_core1;
    uint32_t _last_check_time_core0;
    uint32_t _last_check_time_core1;
    volatile float _latest_usage_core0;
#if CPU_CORE_COUNT > 1
    volatile float _latest_usage_core1;
#endif
    uint32_t _update_interval_ms;
};

#endif // CPU_USAGE_H