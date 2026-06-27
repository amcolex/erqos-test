// test_behaviors.cpp — black-box checks of the sketch with scripted inputs.
//
// We run the real setup()/loop() (compiled from firmware.ino) against the fake
// EQSP32 and assert on the observable outputs: the pump/valve commands and the
// RGB LED, which by spec §9 encodes the operating state. A frozen PlantModel
// keeps our scripted sensor values fixed across loop() iterations.
#include <doctest/doctest.h>
#include "support/harness.h"

// Healthy "at rest" inputs: ample supply, part-full product tank, cool motor,
// no pressure, clean permeate, RUN selected, no feed flow (pump is off).
static void healthy() {
    eqsp32.runSwitch = true;
    eqsp32.setSupplyCm(200);
    eqsp32.setCleanCm(100);
    eqsp32.setMotorC(25);
    eqsp32.setPressureInPsi(0);
    eqsp32.setPressureOutPsi(0);
    eqsp32.setTdsInMgL(3000);
    eqsp32.setTdsOutMgL(50);
    eqsp32.feedPulses = eqsp32.permeatePulses = 0;
}

// Standard frozen-plant fixture: returns a frozen plant; caller scripts inputs.
#define BEGIN_FROZEN() PlantModel plant(eqsp32); plant.frozen = true; \
                       sketchReset(0); healthy(); uint32_t now = 0

TEST_CASE("power-on with RUN starts producing, green LED (spec §6.2)") {
    BEGIN_FROZEN();
    advance(plant, now, 2000);
    CHECK(outProducing());
    CHECK(ledGreen());
}

TEST_CASE("power-on with STOP stays halted, LED off (spec §6.2)") {
    BEGIN_FROZEN();
    eqsp32.runSwitch = false;
    advance(plant, now, 2000);
    CHECK(outStopped());
    CHECK(ledOff());
}

TEST_CASE("supply tank low -> paused, blue blinking (spec §6.8)") {
    BEGIN_FROZEN();
    eqsp32.setSupplyCm(40);
    advance(plant, now, 1000);
    CHECK(outStopped());
    LedObs o = observeLed(plant, now);
    CHECK(o.blue); CHECK(o.off);            // blinking blue
}

TEST_CASE("clean tank full -> stopped, blue solid (spec §6.7)") {
    BEGIN_FROZEN();
    eqsp32.setCleanCm(390);                  // >= restart threshold
    advance(plant, now, 1000);
    CHECK(outStopped());
    LedObs o = observeLed(plant, now);
    CHECK(o.blue); CHECK_FALSE(o.off);       // solid blue
}

TEST_CASE("fault: motor over-temperature, red code 1 (spec §10)") {
    BEGIN_FROZEN();
    eqsp32.setMotorC(85);
    advance(plant, now, 300);
    CHECK(outStopped());
    CHECK(observedFaultCode(plant, now) == 1);
}

TEST_CASE("fault: over-pressure, red code 4 (spec §10)") {
    BEGIN_FROZEN();
    eqsp32.setPressureInPsi(260);
    advance(plant, now, 300);
    CHECK(outStopped());
    CHECK(observedFaultCode(plant, now) == 4);
}

TEST_CASE("fault: 4-20 mA broken wire, red code 5 (spec §10)") {
    BEGIN_FROZEN();
    eqsp32.pressIn_raw = 300;                // ~3 mA
    advance(plant, now, 300);
    CHECK(outStopped());
    CHECK(observedFaultCode(plant, now) == 5);
}

TEST_CASE("fault: NTC open circuit is a sensor fault, red code 5 (spec §10)") {
    BEGIN_FROZEN();
    eqsp32.motorTemp_raw = -9999;            // TIN_OPEN
    advance(plant, now, 300);
    CHECK(outStopped());
    CHECK(observedFaultCode(plant, now) == 5);
}

TEST_CASE("fault: dry-run after the grace time, red code 2 (spec §7/§10)") {
    BEGIN_FROZEN();                           // pump runs but no feed pulses arrive
    advance(plant, now, 8000);
    CHECK(outProducing());                    // still within the 10 s grace
    advance(plant, now, 4000);                // cross the grace time
    CHECK(outStopped());
    CHECK(observedFaultCode(plant, now) == 2);
}

TEST_CASE("emergency stop is immediate (spec §6.1)") {
    BEGIN_FROZEN();
    advance(plant, now, 2000);
    REQUIRE(outProducing());
    eqsp32.runSwitch = false;
    advance(plant, now, 100);
    CHECK(outStopped());
    CHECK(ledOff());
}

TEST_CASE("moving to STOP clears a latched fault (spec §6.1/§10)") {
    BEGIN_FROZEN();
    eqsp32.setMotorC(85);
    advance(plant, now, 300);
    REQUIRE(outStopped());
    REQUIRE(observedFaultCode(plant, now) == 1);

    eqsp32.runSwitch = false;                 // STOP clears the latch
    advance(plant, now, 100);
    CHECK(ledOff());

    eqsp32.setMotorC(25); eqsp32.runSwitch = true;   // clear cause, back to RUN
    advance(plant, now, 1000);
    CHECK(outProducing());                    // restarts (min OFF already satisfied)
}

TEST_CASE("minimum OFF-time gates a restart (spec §6.5)") {
    BEGIN_FROZEN();
    advance(plant, now, 2000);                // producing, pump on
    REQUIRE(outProducing());
    eqsp32.runSwitch = false; advance(plant, now, 100);   // STOP -> OFF timer starts (~2.1 s)
    REQUIRE(outStopped());
    eqsp32.runSwitch = true;

    advance(plant, now, 5000);                // ~7.2 s: < 10 s OFF time -> still off
    CHECK(outStopped());
    advance(plant, now, 6000);                // ~13.2 s: OFF time elapsed -> restart
    CHECK(outProducing());
}

// --- Threshold / boundary checks: lock down the spec §11 setpoints behaviourally
TEST_CASE("setpoint boundaries (spec §11)") {
    SUBCASE("supply >= 50 cm starts; 49 pauses") {
        BEGIN_FROZEN(); eqsp32.setSupplyCm(50); advance(plant, now, 1500);
        CHECK(outProducing());
    }
    SUBCASE("supply 49 cm does not start") {
        BEGIN_FROZEN(); eqsp32.setSupplyCm(49); advance(plant, now, 1500);
        CHECK(outStopped());
    }
    SUBCASE("clean < 380 cm starts; 380 holds full") {
        BEGIN_FROZEN(); eqsp32.setCleanCm(379); advance(plant, now, 1500);
        CHECK(outProducing());
    }
    SUBCASE("clean 380 cm holds (tank full)") {
        BEGIN_FROZEN(); eqsp32.setCleanCm(380); advance(plant, now, 1500);
        CHECK(outStopped());
    }
    SUBCASE("motor 79 C ok; 80 C faults") {
        BEGIN_FROZEN(); eqsp32.setMotorC(79); advance(plant, now, 1500);
        CHECK(outProducing());
    }
    SUBCASE("motor 80 C faults") {
        BEGIN_FROZEN(); eqsp32.setMotorC(80); advance(plant, now, 500);
        CHECK(outStopped()); CHECK(observedFaultCode(plant, now) == 1);
    }
    SUBCASE("pressure 250 PSI ok; 251 trips") {
        BEGIN_FROZEN(); eqsp32.setPressureInPsi(250); advance(plant, now, 1500);
        CHECK(outProducing());
    }
    SUBCASE("pressure 251 PSI trips over-pressure") {
        BEGIN_FROZEN(); eqsp32.setPressureInPsi(251); advance(plant, now, 500);
        CHECK(outStopped()); CHECK(observedFaultCode(plant, now) == 4);
    }
}
