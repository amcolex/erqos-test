// harness.h — drive the sketch's loop() over a virtual clock and observe the
// outputs. The sketch is black-box here: tests assert on the actuator commands
// and the RGB LED (which, per spec §9, encodes the operating state).
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include "sketch.h"
#include "support/plant_model.h"

// --- Instantaneous actuator reads ------------------------------------------
inline bool outProducing() { return eqsp32.pump && eqsp32.supply && !eqsp32.flush; }
inline bool outFlushing()  { return eqsp32.pump && eqsp32.supply && eqsp32.flush; }
inline bool outStopped()   { return !eqsp32.pump && !eqsp32.supply && !eqsp32.flush; }

// --- Instantaneous LED colour ----------------------------------------------
inline bool ledOff()    { return !eqsp32.ledR && !eqsp32.ledG && !eqsp32.ledB; }
inline bool ledRed()    { return  eqsp32.ledR && !eqsp32.ledG && !eqsp32.ledB; }
inline bool ledGreen()  { return !eqsp32.ledR &&  eqsp32.ledG && !eqsp32.ledB; }
inline bool ledBlue()   { return !eqsp32.ledR && !eqsp32.ledG &&  eqsp32.ledB; }
inline bool ledYellow() { return  eqsp32.ledR &&  eqsp32.ledG && !eqsp32.ledB; }

// --- Time stepping: step the plant, then run one loop() at the new time -----
inline void tickStep(PlantModel& plant, uint32_t& now, uint32_t dt) {
    now += dt;
    g_fakeMillis = now;
    plant.step(dt);
    loop();
}
inline void advance(PlantModel& plant, uint32_t& now, uint32_t dur, uint32_t step = 20) {
    uint32_t end = now + dur;
    while ((int32_t)(end - now) > 0) tickStep(plant, now, std::min<uint32_t>(step, end - now));
}
inline bool advanceUntil(PlantModel& plant, uint32_t& now,
                         const std::function<bool()>& pred, uint32_t timeout, uint32_t step = 100) {
    // Step THEN check: the plant must apply inputs and loop() must run at least
    // once before the predicate is meaningful (after sketchReset the outputs are
    // all-off, so "stopped" would be spuriously true at t=0).
    uint32_t end = now + timeout;
    while ((int32_t)(end - now) > 0) {
        tickStep(plant, now, std::min<uint32_t>(step, end - now));
        if (pred()) return true;
    }
    return pred();
}

// --- LED pattern over a window (system assumed steady) ----------------------
struct LedObs { bool green=false, blue=false, yellow=false, red=false, off=false; };
inline LedObs observeLed(PlantModel& plant, uint32_t& now, uint32_t window_ms = 1500, uint32_t step = 25) {
    LedObs o;
    uint32_t end = now + window_ms;
    while ((int32_t)(end - now) > 0) {
        if (ledGreen())  o.green = true;
        if (ledBlue())   o.blue = true;
        if (ledYellow()) o.yellow = true;
        if (ledRed())    o.red = true;
        if (ledOff())    o.off = true;
        tickStep(plant, now, std::min<uint32_t>(step, end - now));
    }
    return o;
}

// Count the red blinks in one fault-code group (spec §10). Syncs to the long gap
// first, then counts rising edges in the next group. The skid is latched in
// FAULT here, so stepping does not change state.
inline int observedFaultCode(PlantModel& plant, uint32_t& now, uint32_t step = 5) {
    auto red = [] { return ledRed(); };
    uint32_t offRun = 0, guard;

    guard = now + 8000;                              // sync: wait for the inter-code gap
    while ((int32_t)(guard - now) > 0) {
        offRun = red() ? 0 : offRun + step;
        if (offRun >= 600) break;
        tickStep(plant, now, step);
    }
    guard = now + 8000;                              // advance to the group start
    while (!red() && (int32_t)(guard - now) > 0) tickStep(plant, now, step);

    int count = 0; bool prev = false; offRun = 0; guard = now + 8000;
    while ((int32_t)(guard - now) > 0) {
        bool on = red();
        if (on && !prev) count++;
        if (!on) { offRun += step; if (offRun >= 500 && count > 0) break; } else offRun = 0;
        prev = on;
        tickStep(plant, now, step);
    }
    return count;
}
