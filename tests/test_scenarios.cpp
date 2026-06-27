// test_scenarios.cpp — closed-loop, end-to-end behaviour over simulated time.
//
// The real setup()/loop() (firmware.ino) runs against the virtual PlantModel,
// which turns pump/valve commands into pressure/flow/level/temperature dynamics
// fed back as sensor readings. Assertions are black-box: actuator commands and
// the RGB LED (which encodes the operating state, spec §9).
#include <doctest/doctest.h>
#include "support/harness.h"

static bool producing() { return outProducing(); }
static bool stopped()   { return outStopped(); }
static bool flushing()  { return outFlushing(); }

TEST_CASE("happy path: power-on -> run, then tank-full hysteresis (spec §6.2/§6.4/§6.7)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 350.0f;   // start near full to keep the test quick
    plant.supplyTank_cm = 200.0f;
    sketchReset(0); uint32_t now = 0;

    REQUIRE(advanceUntil(plant, now, producing, 40000));
    advance(plant, now, 5000);                 // settle flow/pressure
    CHECK(producing());
    {   LedObs o = observeLed(plant, now);     // running in spec = solid green
        CHECK(o.green); CHECK_FALSE(o.off); CHECK_FALSE(o.yellow); }

    REQUIRE(advanceUntil(plant, now, stopped, 20u * 60000u));   // fills to 400 -> stop
    {   LedObs o = observeLed(plant, now);     // tank full = solid blue
        CHECK(o.blue); CHECK_FALSE(o.off); }

    REQUIRE(advanceUntil(plant, now, producing, 20u * 60000u)); // drains < 380 -> restart
    CHECK(producing());
}

TEST_CASE("periodic membrane flush then resume (spec §6.6)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f;
    plant.cleanDraw_lpm = 2.0f;                // balance permeate so the tank never fills
    sketchReset(0); uint32_t now = 0;

    REQUIRE(advanceUntil(plant, now, producing, 40000));
    REQUIRE(advanceUntil(plant, now, flushing, 65u * 60000u));  // after 60 min run
    CHECK(eqsp32.pump); CHECK(eqsp32.supply); CHECK(eqsp32.flush);
    {   LedObs o = observeLed(plant, now);     // flushing = solid yellow
        CHECK(o.yellow); CHECK_FALSE(o.off); }

    REQUIRE(advanceUntil(plant, now, producing, 10u * 60000u)); // flush ends
    CHECK_FALSE(eqsp32.flush);
}

TEST_CASE("supply-low pause and resume with hysteresis (spec §6.8)") {
    PlantModel plant(eqsp32);
    plant.supplyTank_cm = 200.0f; plant.cleanTank_cm = 150.0f;
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, producing, 40000));

    plant.supplyTank_cm = 45.0f;               // below 50 -> pause
    REQUIRE(advanceUntil(plant, now, stopped, 30000));
    {   LedObs o = observeLed(plant, now); CHECK(o.blue); CHECK(o.off); }   // blue blink

    plant.supplyTank_cm = 55.0f;               // < resume threshold 60 -> stay paused
    advance(plant, now, 15000);
    CHECK(stopped());

    plant.supplyTank_cm = 65.0f;               // >= 60 -> resume
    REQUIRE(advanceUntil(plant, now, producing, 40000));
}

TEST_CASE("supply-low during a flush pauses the pump (spec §6.6/§6.8)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.cleanDraw_lpm = 2.0f; plant.supplyTank_cm = 200.0f;
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, producing, 40000));
    REQUIRE(advanceUntil(plant, now, flushing, 65u * 60000u));

    plant.supplyTank_cm = 40.0f;               // feed runs out mid-flush
    REQUIRE(advanceUntil(plant, now, stopped, 20000));
    CHECK_FALSE(eqsp32.flush);
}

TEST_CASE("fault: dry-run (no feed), red code 2 (spec §7/§10)") {
    PlantModel plant(eqsp32);
    plant.blockFeed = true;
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, stopped, 15000));
    CHECK(observedFaultCode(plant, now) == 2);
}

TEST_CASE("fault: over-pressure from heavy fouling, red code 4 (spec §8/§10)") {
    PlantModel plant(eqsp32);
    plant.foulingOffset_psi = 100.0f;          // ~275 PSI > 250 trip
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, stopped, 20000));
    CHECK(observedFaultCode(plant, now) == 4);
}

TEST_CASE("fault: motor over-temperature, red code 1 (spec §10)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.motorRunTemp_c = 85.0f;
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, stopped, 120000));
    CHECK(observedFaultCode(plant, now) == 1);
}

TEST_CASE("fault: broken pressure-sensor wire, red code 5 (spec §10)") {
    PlantModel plant(eqsp32);
    plant.breakPressInWire = true;
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, stopped, 3000));
    CHECK(observedFaultCode(plant, now) == 5);
}

TEST_CASE("fault: permeate TDS over limit for 10 min, red code 3 (spec §7/§10)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.permeateTds_mgL = 250.0f;
    sketchReset(0); uint32_t now = 0;
    advance(plant, now, 35000);                // past the 25 s settle + warn debounce
    CHECK(producing());
    {   LedObs o = observeLed(plant, now); CHECK(o.yellow); CHECK(o.off); }  // TDS-rising warning

    REQUIRE(advanceUntil(plant, now, stopped, 12u * 60000u));
    CHECK(observedFaultCode(plant, now) == 3);
}

TEST_CASE("warning: early fouling (high feed pressure) is warn-only (spec §8)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.foulingOffset_psi = 30.0f;   // ~205 PSI
    sketchReset(0); uint32_t now = 0;
    advance(plant, now, 35000);                // past settle + warn debounce
    CHECK(producing());                        // warn-only: still producing
    LedObs o = observeLed(plant, now);
    CHECK(o.yellow); CHECK(o.off);             // yellow blink
}

TEST_CASE("warning: cleaning required (high pressure + low permeate) (spec §8)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.foulingOffset_psi = 30.0f; plant.recoveryFrac = 0.05f;
    sketchReset(0); uint32_t now = 0;
    advance(plant, now, 35000);                // past settle + warn debounce
    CHECK(producing());
    LedObs o = observeLed(plant, now);
    CHECK(o.yellow); CHECK(o.off);
}

TEST_CASE("production alarms are masked during the start settling window (spec §6.4)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.foulingOffset_psi = 30.0f;  // would warn once running
    sketchReset(0); uint32_t now = 0;

    advance(plant, now, 10000);                // within the 25 s settle
    CHECK(producing());
    {   LedObs o = observeLed(plant, now, 1000);   // masked: solid green, no yellow
        CHECK(o.green); CHECK_FALSE(o.yellow); }

    advance(plant, now, 25000);                // past settle + debounce
    {   LedObs o = observeLed(plant, now); CHECK(o.yellow); }
}

TEST_CASE("fault recovery: STOP clears the latch, then RUN restarts (spec §6.1/§10)") {
    PlantModel plant(eqsp32);
    plant.cleanTank_cm = 100.0f; plant.motorRunTemp_c = 85.0f;
    sketchReset(0); uint32_t now = 0;
    REQUIRE(advanceUntil(plant, now, stopped, 120000));
    REQUIRE(observedFaultCode(plant, now) == 1);

    plant.motorRunTemp_c = 45.0f;              // clear the cause
    eqsp32.runSwitch = false;                  // STOP clears the latch
    advance(plant, now, 500);
    CHECK(ledOff());

    eqsp32.runSwitch = true;
    REQUIRE(advanceUntil(plant, now, producing, 60000));
}
