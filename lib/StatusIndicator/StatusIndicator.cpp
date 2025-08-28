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

// MODIFIED: update() now checks the current mode before building a new sequence.
void StatusIndicator::update(bool is_updating, bool over_voltage, bool under_voltage, bool over_current, int client_count, bool wifi_connected, bool internet_connected)
{
    // Only build a standard status sequence if we are in normal mode.
    if (_needsNewSequence && _currentMode == MODE_NORMAL)
    {
        _buildSequence(is_updating, over_voltage, under_voltage, over_current, client_count, wifi_connected, internet_connected);
    }
    _animate();
}

// NEW: Implementation of the blink method.
void StatusIndicator::blink(uint32_t color, int nTimes, uint32_t blink_duration_ms)
{
    // Switch to override mode to pause the normal status display.
    _currentMode = MODE_OVERRIDE;
    
    _blinkSequence.clear();
    const uint32_t COLOR_BLACK = 0x000000;

    // Build the custom blink sequence.
    for (int i = 0; i < nTimes; ++i) {
        _blinkSequence.push_back({color, blink_duration_ms});
        // Add a pause after each blink, except for the very last one.
        if (i < nTimes - 1) {
            _blinkSequence.push_back({COLOR_BLACK, blink_duration_ms});
        }
    }

    // Immediately start the new sequence.
    _needsNewSequence = false;
    _sequenceIndex = 0;
    _blinkStartTime = millis();
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

// MODIFIED: _animate() now handles switching back to normal mode.
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
            // If the sequence finished and we were in override mode,
            // switch back to normal mode.
            if (_currentMode == MODE_OVERRIDE) {
                _currentMode = MODE_NORMAL;
            }
            
            // Signal that a new sequence (the normal status one) should be built on the next update.
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
    // The fade logic in this function is not ideal for short blinks.
    // For simplicity, we will keep it, but for a fast blink rate,
    // you might want to replace this with a simple on/off.
    if (blink.color == 0x000000)
    {
        strip.setLedColorData(0, 0x000000);
    }
    else
    {
        // This fade logic works fine, but be aware that if blink_duration_ms is
        // less than FADE_IN_DURATION + FADE_OUT_DURATION, it won't reach full brightness.
        float brightness_factor = 0.0f;
        if (elapsed < FADE_IN_DURATION)
        {
            brightness_factor = sin(((float)elapsed / FADE_IN_DURATION) * PI / 2.0f);
        }
        else if (elapsed < blink.duration_ms - FADE_OUT_DURATION) // Adjusted for variable duration
        {
            brightness_factor = 1.0f;
        }
        else
        {
            unsigned long fade_out_start_time = blink.duration_ms - FADE_OUT_DURATION;
            // Ensure we don't get negative values if duration is too short
            if (elapsed > fade_out_start_time) {
                unsigned long fade_out_elapsed = elapsed - fade_out_start_time;
                brightness_factor = cos(((float)fade_out_elapsed / FADE_OUT_DURATION) * PI / 2.0f);
            } else {
                 // If duration is very short, just hold max brightness until fade out
                 brightness_factor = 1.0f;
            }
        }
        uint8_t brightness = 255 * constrain(brightness_factor, 0.0, 1.0);
        strip.setBrightness(brightness);
        strip.setLedColorData(0, blink.color);
    }
    strip.show();
}