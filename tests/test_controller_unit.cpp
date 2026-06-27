// test_controller_unit.cpp — controller logic in isolation (FakeEqsp32 + manual
// clock). Covers power-on behaviour, start permissives, the minimum OFF-time,
// emergency stop / reset, and each immediate safety fault. Sequences that need
// sustained realistic flow (running, hysteresis, flush, TDS) live in
// test_scenarios.cpp where the PlantModel supplies the dynamics.
#include <doctest/doctest.h>
#include "ro_controller.h"
#include "sensors.h"
#include "support/fake_eqsp32.h"
#include "support/harness.h"

using namespace ro;

// A sane "at rest" sensor picture: ample supply, part-full product tank, cool
// motor, no pressure, clean permeate, RUN selected. No feed flow (pump is off).
static void healthy(FakeEqsp32& io) {
    io.runSwitch = true;
    io.setSupplyCm(200);
    io.setCleanCm(100);
    io.setMotorC(25);
    io.setPressureInPsi(0);
    io.setPressureOutPsi(0);
    io.setTdsInMgL(3000);
    io.setTdsOutMgL(50);
    io.feedPulses = io.permeatePulses = 0;
}

TEST_CASE("Params defaults match spec §11") {
    FakeEqsp32 io; RoController c(io);
    const Params& p = c.params();
    CHECK(p.supplyMin_cm == 50.0f);
    CHECK(p.supplyResume_cm == 60.0f);
    CHECK(p.cleanFull_cm == 400.0f);
    CHECK(p.cleanRestart_cm == 380.0f);
    CHECK(p.pressBandLo_psi == 150.0f);
    CHECK(p.pressBandHi_psi == 200.0f);
    CHECK(p.overPressure_psi == 250.0f);
    CHECK(p.tdsLimit_mgL == 200.0f);
    CHECK(p.tdsFaultDelay_ms == 10u * 60u * 1000u);
    CHECK(p.feedFlowLo_lpm == 8.0f);
    CHECK(p.feedFlowHi_lpm == 12.0f);
    CHECK(p.recoveryLo_pct == 10.0f);
    CHECK(p.recoveryHi_pct == 30.0f);
    CHECK(p.motorTempLimit_c == 80.0f);
    CHECK(p.flushInterval_ms == 60u * 60u * 1000u);
    CHECK(p.flushDuration_ms == 5u * 60u * 1000u);
    CHECK(p.minOnTime_ms == 10u * 1000u);
    CHECK(p.minOffTime_ms == 10u * 1000u);
    CHECK(p.settleWindow_ms == 25u * 1000u);
    CHECK(p.dryRunGrace_ms == 10u * 1000u);
}

TEST_CASE("power-on: selector RUN starts automatically (spec §6.2)") {
    FakeEqsp32 io; healthy(io);
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    advance(c, now, 2000);
    CHECK(c.state() == State::STARTING);
    CHECK(io.pump);
    CHECK(io.supply);
    CHECK_FALSE(io.flush);
}

TEST_CASE("power-on: selector STOP stays halted (spec §6.2)") {
    FakeEqsp32 io; healthy(io); io.runSwitch = false;
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    advance(c, now, 2000);
    CHECK(c.state() == State::STOP_ESTOP);
    CHECK_FALSE(io.pump);
    CHECK(c.rgb() == RGB_OFF);
}

TEST_CASE("start permissive: supply tank low -> PAUSED (spec §6.3/§6.8)") {
    FakeEqsp32 io; healthy(io); io.setSupplyCm(40);
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 1000);
    CHECK(c.state() == State::PAUSED_SUPPLY_LOW);
    CHECK_FALSE(io.pump);
}

TEST_CASE("start permissive: clean tank full -> STOPPED (spec §6.3/§6.7)") {
    FakeEqsp32 io; healthy(io); io.setCleanCm(390);  // >= restart threshold
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 1000);
    CHECK(c.state() == State::STOPPED_TANK_FULL);
    CHECK_FALSE(io.pump);
}

TEST_CASE("safety fault: motor over-temperature (spec §10 code 1)") {
    FakeEqsp32 io; healthy(io); io.setMotorC(85);
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 300);
    CHECK(c.state() == State::FAULT);
    CHECK(c.fault() == Fault::MOTOR_OVERTEMP);
    CHECK_FALSE(io.pump);
    CHECK_FALSE(io.supply);
    CHECK_FALSE(io.flush);
}

TEST_CASE("safety fault: over-pressure (spec §10 code 4)") {
    FakeEqsp32 io; healthy(io); io.setPressureInPsi(260);
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 300);
    CHECK(c.state() == State::FAULT);
    CHECK(c.fault() == Fault::OVERPRESSURE);
}

TEST_CASE("safety fault: 4-20 mA broken wire (spec §10 code 5)") {
    FakeEqsp32 io; healthy(io); io.pressIn_raw = 300;  // ~3 mA
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 300);
    CHECK(c.state() == State::FAULT);
    CHECK(c.fault() == Fault::SENSOR);
}

TEST_CASE("safety fault: NTC open circuit is a sensor fault (spec §10 code 5)") {
    FakeEqsp32 io; healthy(io); io.motorTemp_raw = TIN_OPEN;
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 300);
    CHECK(c.state() == State::FAULT);
    CHECK(c.fault() == Fault::SENSOR);
}

TEST_CASE("safety fault: dry-run after the grace time (spec §7/§10 code 2)") {
    FakeEqsp32 io; healthy(io);   // pump will run but no feed pulses are produced
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    advance(c, now, 8000);
    CHECK(c.state() == State::STARTING);     // still within the 10 s grace
    CHECK(c.fault() == Fault::NONE);
    advance(c, now, 4000);                    // cross the grace time
    CHECK(c.state() == State::FAULT);
    CHECK(c.fault() == Fault::DRY_RUN);
    CHECK_FALSE(io.pump);
}

TEST_CASE("emergency stop is immediate (spec §6.1)") {
    FakeEqsp32 io; healthy(io);
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 2000);
    REQUIRE(c.state() == State::STARTING);
    REQUIRE(io.pump);
    io.runSwitch = false;
    advance(c, now, 100);
    CHECK(c.state() == State::STOP_ESTOP);
    CHECK_FALSE(io.pump);
}

TEST_CASE("moving to STOP clears a latched fault (spec §6.1/§10)") {
    FakeEqsp32 io; healthy(io); io.setMotorC(85);
    RoController c(io); c.reset(0);
    uint32_t now = 0; advance(c, now, 300);
    REQUIRE(c.fault() == Fault::MOTOR_OVERTEMP);

    io.runSwitch = false;             // STOP clears the latch
    advance(c, now, 100);
    CHECK(c.state() == State::STOP_ESTOP);
    CHECK(c.fault() == Fault::NONE);

    io.setMotorC(25); io.runSwitch = true;   // clear cause, back to RUN
    advance(c, now, 1000);
    CHECK(c.state() == State::STARTING);      // restarts (min OFF already satisfied)
}

TEST_CASE("minimum OFF-time gates a restart (spec §6.5)") {
    FakeEqsp32 io; healthy(io);
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    advance(c, now, 2000);                    // STARTING, pump on
    REQUIRE(io.pump);
    io.runSwitch = false; advance(c, now, 100);   // STOP -> pump off, OFF timer starts (~2.1 s)
    REQUIRE_FALSE(io.pump);
    io.runSwitch = true;

    advance(c, now, 5000);                    // ~7.2 s: < 10 s OFF time -> still held off
    CHECK_FALSE(io.pump);
    CHECK(c.state() != State::STARTING);

    advance(c, now, 6000);                    // ~13.2 s: OFF time elapsed -> restart
    CHECK(io.pump);
    CHECK(c.state() == State::STARTING);
}
