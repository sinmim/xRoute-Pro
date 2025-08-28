#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

#include <Arduino.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include <vector>

class StatusIndicator
{
public:
    // The StatusColors struct defines all configurable colors.
    struct StatusColors
    {
        uint32_t wifi_disconnected = 0xff0000; // Red
        uint32_t wifi_connected = 0x00ff00;    // Green
        uint32_t client_connected = 0x0000ff;  // Blue
        uint32_t updating = 0xffffff;          // White
        uint32_t over_voltage = 0xff0000;      // Red
        uint32_t under_voltage = 0xffa000;     // Orange
        uint32_t over_current = 0xFF4500;      // OrangeRed
        uint32_t heartbeat = 0xAAAAAA;         // A dim blue for the "alive" blink
        uint32_t orange = 0xffa000;            // Orange
        uint32_t yellow = 0xffff00;            // Yellow
        uint32_t violet = 0x8A2BE2;            // Violet
        uint32_t black = 0x000000;             // Black
    };

    // 'colors' is now a public member. You can change its values directly.
    // Example: led.colors.heartbeat = 0x101010;
    StatusColors colors;

    StatusIndicator(uint8_t pin);
    void begin();
    void update(bool is_updating, bool over_voltage, bool under_voltage, bool over_current, int client_count, bool wifi_connected, bool internet_connected);

    // NEW: Public method to trigger a non-blocking blink sequence.
    // This will pause the normal status indication, run the blink sequence, and then resume.
    // @param color The color to blink (e.g., 0xFF00FF for magenta).
    // @param nTimes The number of times to blink the color.
    // @param blink_duration_ms The duration for both the 'on' and 'off' phase of each blink.
    void blink(uint32_t color, int nTimes, uint32_t blink_duration_ms);

private:
    struct Blink
    {
        uint32_t color;
        uint32_t duration_ms;
    };

    // NEW: An enum to manage the indicator's current operational mode.
    enum IndicatorMode
    {
        MODE_NORMAL,  // Default behavior, showing system status.
        MODE_OVERRIDE // Temporary override for custom blinks.
    };

    Freenove_ESP32_WS2812 strip;

    std::vector<Blink> _blinkSequence;
    int _sequenceIndex = 0;
    unsigned long _blinkStartTime = 0;
    bool _needsNewSequence = true;

    // NEW: Member variable to store the current mode.
    IndicatorMode _currentMode = MODE_NORMAL;

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
    void _renderBlink(const Blink &blink, unsigned long elapsed);
};

#endif // STATUS_INDICATOR_H