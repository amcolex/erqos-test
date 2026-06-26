// rgb_indicator.cpp — implementation of the RGB annunciation logic.
#include "rgb_indicator.h"

namespace ro {

// 50%-duty square wave: true for the first half of each period.
static bool blink5050(uint32_t now_ms, uint32_t period_ms) {
    if (period_ms == 0) return true;
    return (now_ms % period_ms) < (period_ms / 2);
}

bool faultCodeOn(Fault fault, uint32_t now_ms, const RgbTiming& t) {
    int code = static_cast<int>(fault);
    if (code <= 0) return false;
    uint32_t unit = t.faultBlinkOn_ms + t.faultBlinkOff_ms;          // one blink cell
    uint32_t cycle = code * unit + t.faultGap_ms;                    // N blinks + gap
    if (cycle == 0) return true;
    uint32_t phase = now_ms % cycle;
    if (phase >= (uint32_t)code * unit) return false;               // in the gap
    return (phase % unit) < t.faultBlinkOn_ms;                      // on portion of a blink
}

Rgb resolveRgb(State state, Fault fault, Warnings warn, uint32_t now_ms,
               const RgbTiming& t) {
    // Priority, highest to lowest (spec §9):
    // STOP(off) > Fault(red code) > Flushing(yellow solid) > Warning(yellow blink)
    // > Running(green) > Supply-low(blue blink) > Tank-full(blue solid).
    switch (state) {
        case State::STOP_ESTOP:
            return RGB_OFF;

        case State::FAULT:
            return faultCodeOn(fault, now_ms, t) ? RGB_RED : RGB_OFF;

        case State::FLUSHING:
            return RGB_YELLOW;  // solid

        case State::STARTING:
            return RGB_GREEN;   // spec §5: STARTING = green

        case State::RUNNING:
            if (warn.any()) {
                // Sub-priority: fast (TDS/clean) > medium (flow/rec) > slow (pressure).
                uint32_t period = warn.tdsOrClean ? t.warnFast_ms
                                : warn.flowOrRec  ? t.warnMedium_ms
                                                  : t.warnSlow_ms;
                return blink5050(now_ms, period) ? RGB_YELLOW : RGB_OFF;
            }
            return RGB_GREEN;   // solid

        case State::PAUSED_SUPPLY_LOW:
            return blink5050(now_ms, t.blueBlink_ms) ? RGB_BLUE : RGB_OFF;

        case State::STOPPED_TANK_FULL:
            return RGB_BLUE;    // solid
    }
    return RGB_OFF;
}

} // namespace ro
