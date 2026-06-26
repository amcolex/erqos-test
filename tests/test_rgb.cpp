// test_rgb.cpp — RGB priority resolution + blink/fault-code patterns (spec §9/§10).
#include <doctest/doctest.h>
#include "rgb_indicator.h"

using namespace ro;

static Warnings noWarn() { return Warnings{}; }

TEST_CASE("state -> colour priority (spec §9)") {
    RgbTiming t;
    CHECK(resolveRgb(State::STOP_ESTOP, Fault::NONE, noWarn(), 0, t) == RGB_OFF);
    CHECK(resolveRgb(State::STARTING,   Fault::NONE, noWarn(), 0, t) == RGB_GREEN);
    CHECK(resolveRgb(State::RUNNING,    Fault::NONE, noWarn(), 0, t) == RGB_GREEN);
    CHECK(resolveRgb(State::FLUSHING,   Fault::NONE, noWarn(), 0, t) == RGB_YELLOW);   // solid
    CHECK(resolveRgb(State::STOPPED_TANK_FULL, Fault::NONE, noWarn(), 0, t) == RGB_BLUE); // solid
}

TEST_CASE("flushing is solid yellow across time (no off phase)") {
    RgbTiming t;
    for (uint32_t now = 0; now < 3000; now += 50)
        CHECK(resolveRgb(State::FLUSHING, Fault::NONE, noWarn(), now, t) == RGB_YELLOW);
}

TEST_CASE("tank-full is solid blue across time") {
    RgbTiming t;
    for (uint32_t now = 0; now < 3000; now += 50)
        CHECK(resolveRgb(State::STOPPED_TANK_FULL, Fault::NONE, noWarn(), now, t) == RGB_BLUE);
}

TEST_CASE("supply-low blinks blue (on and off observed)") {
    RgbTiming t;
    bool on = false, off = false;
    for (uint32_t now = 0; now < 2000; now += 25) {
        Rgb c = resolveRgb(State::PAUSED_SUPPLY_LOW, Fault::NONE, noWarn(), now, t);
        if (c == RGB_BLUE) on = true;
        if (c == RGB_OFF)  off = true;
    }
    CHECK(on);
    CHECK(off);
}

TEST_CASE("running+warning blinks yellow at the warning's rate (spec §9)") {
    RgbTiming t;
    auto period = [&](Warnings w) {
        // measure the on->off->on cycle by finding first off then next on
        bool sawOn = false, sawOff = false;
        for (uint32_t now = 0; now < 3000; now += 5) {
            Rgb c = resolveRgb(State::RUNNING, Fault::NONE, w, now, t);
            if (c == RGB_YELLOW) sawOn = true;
            if (c == RGB_OFF)    sawOff = true;
        }
        return sawOn && sawOff;
    };
    Warnings press;  press.pressure = true;       // slow
    Warnings flow;   flow.flowOrRec = true;       // medium
    Warnings tds;    tds.tdsOrClean = true;       // fast
    CHECK(period(press));
    CHECK(period(flow));
    CHECK(period(tds));

    SUBCASE("fast (TDS/clean) wins when several warnings coincide") {
        Warnings all; all.pressure = all.flowOrRec = all.tdsOrClean = true;
        // At t = warnFast/2 the fast blink is OFF; the slow blink would still be ON.
        uint32_t tHalfFast = t.warnFast_ms / 2 + 1;
        CHECK(resolveRgb(State::RUNNING, Fault::NONE, all, tHalfFast, t) == RGB_OFF);
    }
}

TEST_CASE("fault shows N red blinks matching the fault code (spec §10)") {
    RgbTiming t;
    auto countBlinks = [&](Fault f) {
        // Count rising edges of the red LED over exactly one full code cycle.
        int code = (int)f;
        uint32_t unit = t.faultBlinkOn_ms + t.faultBlinkOff_ms;
        uint32_t cycle = code * unit + t.faultGap_ms;
        int blinks = 0; bool prev = false;
        for (uint32_t now = 0; now < cycle; now += 5) {
            bool on = faultCodeOn(f, now, t);
            if (on && !prev) ++blinks;
            prev = on;
        }
        return blinks;
    };
    CHECK(countBlinks(Fault::MOTOR_OVERTEMP) == 1);
    CHECK(countBlinks(Fault::DRY_RUN)        == 2);
    CHECK(countBlinks(Fault::PERMEATE_TDS)   == 3);
    CHECK(countBlinks(Fault::OVERPRESSURE)   == 4);
    CHECK(countBlinks(Fault::SENSOR)         == 5);
}

TEST_CASE("fault LED colour is red (when on) regardless of warnings") {
    RgbTiming t;
    Warnings w; w.pressure = w.flowOrRec = w.tdsOrClean = true;
    // Sample a moment where the code is on.
    Rgb c = resolveRgb(State::FAULT, Fault::MOTOR_OVERTEMP, w, 0, t);
    CHECK(c == RGB_RED);
}
