#include "CPU_Usage.h"
#include "esp_freertos_hooks.h"

// Initialize static member variables
volatile uint32_t CPU_Usage::_idle_task_counter_core0 = 0;
volatile uint32_t CPU_Usage::_idle_task_counter_core1 = 0;

CPU_Usage::CPU_Usage() {
    _monitoring_task_handle = NULL;
    _baseline_idle_count_per_sec_core0 = 0.0f;
    _baseline_idle_count_per_sec_core1 = 0.0f;
    _last_check_time_core0 = 0;
    _last_check_time_core1 = 0;
    _latest_usage_core0 = 0.0f;
    _latest_usage_core1 = 0.0f;
    _update_interval_ms = 1000;
}

void CPU_Usage::begin(uint32_t calibration_duration_ms, uint32_t update_interval_ms) {
    _update_interval_ms = update_interval_ms;
    
    // This now correctly passes functions that return a bool
    esp_register_freertos_idle_hook_for_cpu(idle_hook_core0, 0);
#if CPU_CORE_COUNT > 1
    esp_register_freertos_idle_hook_for_cpu(idle_hook_core1, 1);
#endif

    Serial.println("Calibrating CPU idle baseline...");

    uint32_t start_time = millis();
    while (millis() - start_time < calibration_duration_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float elapsed_seconds = calibration_duration_ms / 1000.0f;
    _baseline_idle_count_per_sec_core0 = (float)_idle_task_counter_core0 / elapsed_seconds;
    
    Serial.printf("Core 0 baseline: %.2f idle ticks/sec\n", _baseline_idle_count_per_sec_core0);
#if CPU_CORE_COUNT > 1
    _baseline_idle_count_per_sec_core1 = (float)_idle_task_counter_core1 / elapsed_seconds;
    Serial.printf("Core 1 baseline: %.2f idle ticks/sec\n", _baseline_idle_count_per_sec_core1);
#endif
    Serial.println("Calibration complete.");

    _idle_task_counter_core0 = 0;
    _idle_task_counter_core1 = 0;
    _last_check_time_core0 = millis();
    _last_check_time_core1 = millis();

    xTaskCreatePinnedToCore(
        monitoring_task_wrapper, "CPU_Monitor_Task", 2048, this, 5, &_monitoring_task_handle, 0);
}

void CPU_Usage::monitoring_task_wrapper(void* _this) {
    static_cast<CPU_Usage*>(_this)->monitoring_task_body();
}

void CPU_Usage::monitoring_task_body() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(_update_interval_ms));
        update_cpu_stats();
    }
}

void CPU_Usage::update_cpu_stats() {
    // --- Calculate for Core 0 ---
    uint32_t elapsed_time_0 = millis() - _last_check_time_core0;
    if (elapsed_time_0 > 0 && _baseline_idle_count_per_sec_core0 > 0) {
        uint32_t current_idle_count_0 = _idle_task_counter_core0;
        _idle_task_counter_core0 = 0;
        _last_check_time_core0 = millis();

        float current_idle_rate = (float)current_idle_count_0 / (elapsed_time_0 / 1000.0f);
        float idle_percentage = (current_idle_rate / _baseline_idle_count_per_sec_core0) * 100.0f;
        _latest_usage_core0 = 100.0f - idle_percentage;
        if (_latest_usage_core0 < 0.0f) _latest_usage_core0 = 0.0f;
        if (_latest_usage_core0 > 100.0f) _latest_usage_core0 = 100.0f;
    }

    // --- Calculate for Core 1 ---
#if CPU_CORE_COUNT > 1
    uint32_t elapsed_time_1 = millis() - _last_check_time_core1;
    if (elapsed_time_1 > 0 && _baseline_idle_count_per_sec_core1 > 0) {
        uint32_t current_idle_count_1 = _idle_task_counter_core1;
        _idle_task_counter_core1 = 0;
        _last_check_time_core1 = millis();
        
        float current_idle_rate = (float)current_idle_count_1 / (elapsed_time_1 / 1000.0f);
        float idle_percentage = (current_idle_rate / _baseline_idle_count_per_sec_core1) * 100.0f;
        _latest_usage_core1 = 100.0f - idle_percentage;
        if (_latest_usage_core1 < 0.0f) _latest_usage_core1 = 0.0f;
        if (_latest_usage_core1 > 100.0f) _latest_usage_core1 = 100.0f;
    }
#endif
}

// --- FIX: Changed return type and added return statement ---
bool CPU_Usage::idle_hook_core0() { 
    _idle_task_counter_core0++; 
    return true;
}

// --- FIX: Changed return type and added return statement ---
bool CPU_Usage::idle_hook_core1() { 
    _idle_task_counter_core1++; 
    return true;
}

float CPU_Usage::get_cpu_usage_core0() { return _latest_usage_core0; }

#if CPU_CORE_COUNT > 1
float CPU_Usage::get_cpu_usage_core1() { return _latest_usage_core1; }
float CPU_Usage::get_cpu_usage_total() { return (_latest_usage_core0 + _latest_usage_core1) / 2.0f; }
#endif