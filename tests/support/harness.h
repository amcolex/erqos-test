// harness.h — time-advancing helpers for controller + plant scenario tests.
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include "ro_controller.h"
#include "support/fake_eqsp32.h"
#include "support/plant_model.h"

namespace ro {

// Advance controller-only for duration_ms (the test holds the FakeEqsp32 fixed).
inline uint32_t advance(RoController& c, uint32_t& now,
                        uint32_t duration_ms, uint32_t step_ms = 20) {
    uint32_t end = now + duration_ms;
    while ((int32_t)(end - now) > 0) {
        uint32_t s = std::min<uint32_t>(step_ms, end - now);
        now += s;
        c.tick(now);
    }
    return now;
}

// Advance closed-loop: step the plant, then tick the controller.
inline uint32_t advance(RoController& c, PlantModel& p, uint32_t& now,
                        uint32_t duration_ms, uint32_t step_ms = 20) {
    uint32_t end = now + duration_ms;
    while ((int32_t)(end - now) > 0) {
        uint32_t s = std::min<uint32_t>(step_ms, end - now);
        now += s;
        p.step(s);
        c.tick(now);
    }
    return now;
}

// Closed-loop advance until pred(c) holds or timeout elapses. Returns pred state.
inline bool advanceUntil(RoController& c, PlantModel& p, uint32_t& now,
                         const std::function<bool(const RoController&)>& pred,
                         uint32_t timeout_ms, uint32_t step_ms = 100) {
    uint32_t end = now + timeout_ms;
    while ((int32_t)(end - now) > 0) {
        if (pred(c)) return true;
        uint32_t s = std::min<uint32_t>(step_ms, end - now);
        now += s;
        p.step(s);
        c.tick(now);
    }
    return pred(c);
}

// True if, over the next window, the LED is observed both ON and OFF for the
// given colour predicate — i.e. it is blinking (not solid / not dark).
inline bool observeBlink(RoController& c, PlantModel& p, uint32_t& now,
                         const std::function<bool(Rgb)>& isColor,
                         uint32_t window_ms = 2000, uint32_t step_ms = 25) {
    bool sawOn = false, sawOff = false;
    uint32_t end = now + window_ms;
    while ((int32_t)(end - now) > 0) {
        Rgb led = c.rgb();
        if (isColor(led)) sawOn = true;
        if (led == Rgb(false, false, false)) sawOff = true;
        now += step_ms;
        p.step(step_ms);
        c.tick(now);
    }
    return sawOn && sawOff;
}

} // namespace ro
