#include "StatusIndicator.h"

#define LEDS_COUNT 1
#define CHANNEL 0

StatusIndicator::StatusIndicator(uint8_t pin) : strip(LEDS_COUNT, pin, CHANNEL, TYPE_GRB) {}

void StatusIndicator::begin()
{
    strip.begin();
    strip.setBrightness(255);
    strip.show();
}

// The 'colors' parameter has been removed from the function signature.
void StatusIndicator::update(bool is_updating, bool over_voltage, bool under_voltage, bool over_current, int client_count, bool wifi_connected, bool internet_connected)
{
    if (_needsNewSequence)
    {
        // It now calls _buildSequence without the colors parameter.
        _buildSequence(is_updating, over_voltage, under_voltage, over_current, client_count, wifi_connected, internet_connected);
    }
    _animate();
}

// This function now uses the 'colors' member variable that belongs to the class instance.
void StatusIndicator::_buildSequence(bool is_updating, bool over_voltage, bool under_voltage, bool over_current, int client_count, bool wifi_connected, bool internet_connected) {
    _blinkSequence.clear();
    const uint32_t COLOR_BLACK = 0x000000;

    // --- High Priority Events First ---
    // Display the most critical alert and stop.
    if (is_updating) {
        _blinkSequence.push_back({colors.updating, BLINK_DURATION});
    } else if (over_voltage) {
        _blinkSequence.push_back({colors.over_voltage, BLINK_DURATION});
    } else if (over_current) {
        _blinkSequence.push_back({colors.over_current, BLINK_DURATION});
    } else if (under_voltage) {
        _blinkSequence.push_back({colors.under_voltage, BLINK_DURATION});
    }
    
    // --- If no critical alerts, check normal connection status ---
    if (_blinkSequence.empty()) {
        if (client_count > 0) {
            _blinkSequence.push_back({colors.wifi_connected, BLINK_DURATION});
            _blinkSequence.push_back({COLOR_BLACK, MEDIUM_PAUSE});
            for (int i = 0; i < client_count; ++i) {
                _blinkSequence.push_back({colors.client_connected, BLINK_DURATION});
                _blinkSequence.push_back({COLOR_BLACK, SHORT_PAUSE});
            }
        } else if (wifi_connected) {
            _blinkSequence.push_back({colors.wifi_connected, BLINK_DURATION});
        }
    }
    
    // --- If the sequence is STILL empty, it means we are truly idle ---
    if (_blinkSequence.empty()) {
        _blinkSequence.push_back({colors.heartbeat, BLINK_DURATION});
    }

    // --- Finalize the sequence with a long pause at the end ---
    if (!_blinkSequence.empty()) {
        // *** THIS IS THE CORRECTED LOGIC ***
        // If the last blink is a pause, replace it with the long pause.
        if (_blinkSequence.back().color == COLOR_BLACK) {
            _blinkSequence.back().duration_ms = LONG_PAUSE;
        } 
        // Otherwise, it's a single-blink sequence (like heartbeat), so ADD the long pause after it.
        else {
            _blinkSequence.push_back({COLOR_BLACK, LONG_PAUSE});
        }
    }

    _needsNewSequence = false;
    _sequenceIndex = 0;
    _blinkStartTime = millis();
}

// _animate() and _renderBlink() functions do not need any changes.
void StatusIndicator::_animate()
{
    if (_blinkSequence.empty() || _needsNewSequence)
        return;
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - _blinkStartTime;
    const Blink &currentBlink = _blinkSequence[_sequenceIndex];
    if (elapsed >= currentBlink.duration_ms)
    {
        _sequenceIndex++;
        _blinkStartTime = currentTime;
        if (_sequenceIndex >= _blinkSequence.size())
        {
            _needsNewSequence = true;
            return;
        }
        elapsed = 0;
        _renderBlink(_blinkSequence[_sequenceIndex], elapsed);
    }
    else
    {
        _renderBlink(currentBlink, elapsed);
    }
}

void StatusIndicator::_renderBlink(const Blink &blink, unsigned long elapsed)
{
    if (blink.color == 0x000000)
    {
        strip.setLedColorData(0, 0x000000);
    }
    else
    {
        float brightness_factor = 0.0f;
        if (elapsed < FADE_IN_DURATION)
        {
            brightness_factor = sin(((float)elapsed / FADE_IN_DURATION) * PI / 2.0f);
        }
        else if (elapsed < FADE_IN_DURATION + HOLD_DURATION)
        {
            brightness_factor = 1.0f;
        }
        else
        {
            unsigned long fade_out_elapsed = elapsed - (FADE_IN_DURATION + HOLD_DURATION);
            brightness_factor = cos(((float)fade_out_elapsed / FADE_OUT_DURATION) * PI / 2.0f);
        }
        uint8_t brightness = 255 * constrain(brightness_factor, 0.0, 1.0);
        strip.setBrightness(brightness);
        strip.setLedColorData(0, blink.color);
    }
    strip.show();
}