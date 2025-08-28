#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

#include <Arduino.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include <vector>

class StatusIndicator {
public:
    // The StatusColors struct defines all configurable colors.
    struct StatusColors {
        uint32_t wifi_disconnected = 0xff0000; // Red
        uint32_t wifi_connected    = 0x00ff00; // Green
        uint32_t client_connected  = 0x0000ff; // Blue
        uint32_t updating          = 0xffffff; // White
        uint32_t over_voltage      = 0xff0000; // Red
        uint32_t under_voltage     = 0xffa000; // Orange
        uint32_t over_current      = 0xFF4500; // OrangeRed
        uint32_t heartbeat         = 0xAAAAAA; // A dim blue for the "alive" blink
    };

    // 'colors' is now a public member. You can change its values directly.
    // Example: led.colors.heartbeat = 0x101010;
    StatusColors colors;

    StatusIndicator(uint8_t pin);
    void begin();
    // The update function is now simpler to call.
    void update(bool is_updating, bool over_voltage, bool under_voltage, bool over_current, int client_count, bool wifi_connected, bool internet_connected);

private:
    struct Blink {
        uint32_t color;
        uint32_t duration_ms;
    };

    Freenove_ESP32_WS2812 strip;
    
    std::vector<Blink> _blinkSequence;
    int _sequenceIndex = 0;
    unsigned long _blinkStartTime = 0;
    bool _needsNewSequence = true;

    // Default timings
    static const uint32_t FADE_IN_DURATION = 150;
    static const uint32_t HOLD_DURATION = 50;
    static const uint32_t FADE_OUT_DURATION = 150;
    static const uint32_t BLINK_DURATION = FADE_IN_DURATION + HOLD_DURATION + FADE_OUT_DURATION;
    static const uint32_t SHORT_PAUSE = 100;
    static const uint32_t MEDIUM_PAUSE = 200;
    static const uint32_t LONG_PAUSE = 500;

    void _buildSequence(bool is_updating, bool over_voltage, bool under_voltage, bool over_current, int client_count, bool wifi_connected, bool internet_connected);
    void _animate();
    void _renderBlink(const Blink& blink, unsigned long elapsed);
};

#endif // STATUS_INDICATOR_H