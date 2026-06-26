// rgb_indicator.h — pure RGB annunciation logic (spec §9 / §10 blink codes).
//
// Given the current state/fault/warnings and the current time, returns the LED
// colour to display *at this instant*. Blinking is computed from now_ms so the
// caller just samples it every tick. PLATFORM-INDEPENDENT.
#pragma once

#include <cstdint>
#include "ro_types.h"

namespace ro {

// Blink timing. Defaults: slow 1 Hz, medium 2 Hz, fast 4 Hz; fault codes are
// N short blinks (on/off) then a gap, repeating (spec §10). All configurable —
// spec §13 flags flash-rate mapping as an open point.
struct RgbTiming {
    uint32_t blueBlink_ms   = 1000;  // supply-low blue blink full period
    uint32_t warnSlow_ms    = 1000;  // pressure / early fouling
    uint32_t warnMedium_ms  = 500;   // flow / recovery
    uint32_t warnFast_ms    = 250;   // TDS rising / cleaning required
    uint32_t faultBlinkOn_ms  = 200; // fault code: each blink on time
    uint32_t faultBlinkOff_ms = 200; // fault code: each blink off time
    uint32_t faultGap_ms      = 1000;// pause between code repetitions
};

// Colours.
constexpr Rgb RGB_OFF    {false, false, false};
constexpr Rgb RGB_RED    {true,  false, false};
constexpr Rgb RGB_GREEN  {false, true,  false};
constexpr Rgb RGB_BLUE   {false, false, true};
constexpr Rgb RGB_YELLOW {true,  true,  false};

// Resolve the instantaneous LED colour.
Rgb resolveRgb(State state, Fault fault, Warnings warn, uint32_t now_ms,
               const RgbTiming& t = {});

// Helper exposed for testing: is a fault blink-code "on" at this instant?
bool faultCodeOn(Fault fault, uint32_t now_ms, const RgbTiming& t = {});

} // namespace ro
