// test_scenarios.cpp — closed-loop, end-to-end behaviour over simulated time.
//
// Each test runs the real controller against the virtual PlantModel, advancing a
// simulated clock. This mirrors how the skid actually behaves: pump/valve
// commands drive pressure/flow/level/temperature dynamics, which feed back as
// sensor readings. Timeouts and tolerances keep the assertions robust.
#include <doctest/doctest.h>
#include "ro_controller.h"
#include "support/fake_io.h"
#include "support/plant_model.h"
#include "support/harness.h"

using namespace ro;

static bool isRunning(const RoController& c)   { return c.state() == State::RUNNING; }
static bool isFault(const RoController& c)     { return c.state() == State::FAULT; }
static bool isTankFull(const RoController& c)  { return c.state() == State::STOPPED_TANK_FULL; }
static bool isSupplyLow(const RoController& c) { return c.state() == State::PAUSED_SUPPLY_LOW; }
static bool isFlushing(const RoController& c)  { return c.state() == State::FLUSHING; }
static bool isYellow(Rgb c) { return c == RGB_YELLOW; }

TEST_CASE("happy path: power-on -> start -> run, then tank-full hysteresis (spec §6.2/§6.4/§6.7)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 350.0f;   // start near full to keep the test quick
    plant.supplyTank_cm = 200.0f;
    RoController c(io); c.reset(0);
    uint32_t now = 0;

    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));
    advance(c, plant, now, 5000);  // let flow/pressure settle

    CHECK(io.pump);
    CHECK(io.supply);
    CHECK_FALSE(io.flush);
    CHECK_FALSE(c.warnings().any());
    CHECK(c.rgb() == RGB_GREEN);
    CHECK(c.readings().feedFlow_lpm == doctest::Approx(10.0f).epsilon(0.1));
    CHECK(c.readings().recovery_pct == doctest::Approx(20.0f).epsilon(0.2));
    CHECK(c.readings().pressIn_psi == doctest::Approx(175.0f).epsilon(0.05));

    // Product tank fills to 400 cm -> stop (blue solid).
    REQUIRE(advanceUntil(c, plant, now, isTankFull, 20u * 60000u));
    CHECK_FALSE(io.pump);
    CHECK_FALSE(io.supply);
    CHECK(c.rgb() == RGB_BLUE);

    // Drains to < 380 cm -> restarts automatically.
    REQUIRE(advanceUntil(c, plant, now, isRunning, 20u * 60000u));
    CHECK(io.pump);
}

TEST_CASE("periodic membrane flush then resume (spec §6.6)") {
    Params p;
    p.settleWindow_ms = 1000;
    p.flushInterval_ms = 2u * 60000u;   // scaled for test speed (default 60 min)
    p.flushDuration_ms = 30u * 1000u;   // scaled (default 5 min)
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.cleanDraw_lpm = 2.0f;         // balance permeate so the tank never fills
    RoController c(io, p); c.reset(0);
    uint32_t now = 0;

    REQUIRE(advanceUntil(c, plant, now, isRunning, 10000));

    REQUIRE(advanceUntil(c, plant, now, isFlushing, 3u * 60000u));
    CHECK(io.pump);          // pump keeps running during a flush
    CHECK(io.supply);
    CHECK(io.flush);         // brine bypass open
    CHECK(c.rgb() == RGB_YELLOW);

    // Flush completes -> back to RUNNING with the flush valve closed.
    REQUIRE(advanceUntil(c, plant, now, isRunning, 60000));
    CHECK_FALSE(io.flush);
}

TEST_CASE("supply-low during a flush pauses the pump (spec §6.6/§6.8)") {
    Params p;
    p.settleWindow_ms = 1000;
    p.flushInterval_ms = 2u * 60000u;
    p.flushDuration_ms = 60u * 1000u;   // long enough to interrupt mid-flush
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f; plant.cleanDraw_lpm = 2.0f; plant.supplyTank_cm = 200.0f;
    RoController c(io, p); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 10000));
    REQUIRE(advanceUntil(c, plant, now, isFlushing, 3u * 60000u));

    plant.supplyTank_cm = 40.0f;        // feed runs out during the flush
    REQUIRE(advanceUntil(c, plant, now, isSupplyLow, 20000));
    CHECK_FALSE(io.pump);
    CHECK_FALSE(io.flush);
}

TEST_CASE("supply-low pause and resume with hysteresis (spec §6.8)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.supplyTank_cm = 200.0f; plant.cleanTank_cm = 150.0f;
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));

    // Supply falls below 50 cm -> pause (blue blink), pump off.
    plant.supplyTank_cm = 45.0f;
    REQUIRE(advanceUntil(c, plant, now, isSupplyLow, 30000));
    CHECK_FALSE(io.pump);

    // Recovering to 55 cm (< resume threshold 60) must NOT resume.
    plant.supplyTank_cm = 55.0f;
    advance(c, plant, now, 15000);
    CHECK(c.state() == State::PAUSED_SUPPLY_LOW);

    // Reaching 65 cm (>= 60) resumes.
    plant.supplyTank_cm = 65.0f;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));
    CHECK(io.pump);
}

TEST_CASE("minimum ON-time defers a tank-full stop (spec §6.5)") {
    Params p;
    p.settleWindow_ms = 500;
    p.minOnTime_ms = 10000;
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 370.0f;        // starts below the 380 restart threshold
    plant.recoveryFrac = 0.5f;          // strong permeate so the tank fills fast
    plant.cleanDraw_lpm = 0.0f;
    plant.cleanLitersPerCm = 0.005f;    // tank crosses 400 within a couple of seconds
    RoController c(io, p); c.reset(0);
    uint32_t now = 0;

    REQUIRE(advanceUntil(c, plant, now, isRunning, 5000));
    // Tank becomes full well before the min ON-time elapses, but we keep running.
    advance(c, plant, now, 6000);       // ~6-7 s total pump-on, still < 10 s min ON
    CHECK(c.readings().cleanTank_cm >= 400.0f);
    CHECK(c.state() == State::RUNNING);
    // After the min ON-time, the tank-full stop is allowed.
    REQUIRE(advanceUntil(c, plant, now, isTankFull, 10000));
}

TEST_CASE("fault: dry-run (no feed) latches and stops (spec §10 code 2)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.blockFeed = true;             // no feed water
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isFault, 15000));
    CHECK(c.fault() == Fault::DRY_RUN);
    CHECK_FALSE(io.pump);
}

TEST_CASE("fault: over-pressure from heavy fouling (spec §8/§10 code 4)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.foulingOffset_psi = 100.0f;   // ~275 PSI producing -> over the 250 trip
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isFault, 20000));
    CHECK(c.fault() == Fault::OVERPRESSURE);
    CHECK_FALSE(io.pump);
}

TEST_CASE("fault: motor over-temperature (spec §10 code 1)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.motorRunTemp_c = 85.0f;       // climbs past the 80 C limit while running
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isFault, 120000));
    CHECK(c.fault() == Fault::MOTOR_OVERTEMP);
    CHECK_FALSE(io.pump);
}

TEST_CASE("fault: broken pressure-sensor wire (spec §10 code 5)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.breakPressInWire = true;      // ~0 mA on the loop
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isFault, 3000));
    CHECK(c.fault() == Fault::SENSOR);
}

TEST_CASE("fault: permeate TDS over limit for 10 min (spec §7/§10 code 3)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.permeateTds_mgL = 250.0f;     // out of spec
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));

    // First it raises the rising-TDS warning (fast yellow), still running.
    advance(c, plant, now, 5000);
    CHECK(c.state() == State::RUNNING);
    CHECK(c.warnings().tdsOrClean);

    // Sustained over the limit for the fault delay -> latched stop.
    REQUIRE(advanceUntil(c, plant, now, isFault, 12u * 60000u));
    CHECK(c.fault() == Fault::PERMEATE_TDS);
    CHECK_FALSE(io.pump);
}

TEST_CASE("warning: early fouling = high feed pressure, warn-only (spec §8)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.foulingOffset_psi = 30.0f;    // ~205 PSI: above 200, below the 250 trip
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));
    advance(c, plant, now, 5000);       // past the warning debounce

    CHECK(c.state() == State::RUNNING); // warn-only: still producing
    CHECK(c.warnings().pressure);
    CHECK_FALSE(c.warnings().flowOrRec);
    CHECK_FALSE(c.warnings().tdsOrClean);
    CHECK(observeBlink(c, plant, now, isYellow));   // yellow blink, not solid
}

TEST_CASE("warning: cleaning required = high pressure + low permeate (spec §8)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.foulingOffset_psi = 30.0f;    // ~205 PSI
    plant.recoveryFrac = 0.05f;         // permeate ~0.5 L/min (< 1 = "low")
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));
    advance(c, plant, now, 5000);

    CHECK(c.state() == State::RUNNING);
    CHECK(c.warnings().tdsOrClean);     // cleaning-required -> fast yellow
}

TEST_CASE("warning: recovery out of band, warn-only (spec §7)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.recoveryFrac = 0.05f;         // 5% recovery < 10% band; pressure normal
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));
    advance(c, plant, now, 5000);

    CHECK(c.state() == State::RUNNING);
    CHECK(c.warnings().flowOrRec);
    CHECK_FALSE(c.warnings().pressure);
    CHECK_FALSE(c.warnings().tdsOrClean);
}

TEST_CASE("production alarms are masked during the start settling window (spec §6.4)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.foulingOffset_psi = 30.0f;    // would warn once running
    RoController c(io); c.reset(0);
    uint32_t now = 0;

    // During STARTING (within the 25 s settle), no warnings even though pressure
    // is already out of band. Safety items remain active (tested elsewhere).
    advance(c, plant, now, 10000);
    REQUIRE(c.state() == State::STARTING);
    CHECK_FALSE(c.warnings().any());

    // Once RUNNING and past the debounce, the warning surfaces.
    REQUIRE(advanceUntil(c, plant, now, isRunning, 40000));
    advance(c, plant, now, 5000);
    CHECK(c.warnings().pressure);
}

TEST_CASE("fault recovery: STOP clears the latch, then RUN restarts (spec §6.1/§10)") {
    FakeSkidIO io; PlantModel plant(io);
    plant.cleanTank_cm = 100.0f;
    plant.motorRunTemp_c = 85.0f;
    RoController c(io); c.reset(0);
    uint32_t now = 0;
    REQUIRE(advanceUntil(c, plant, now, isFault, 120000));
    REQUIRE(c.fault() == Fault::MOTOR_OVERTEMP);

    // Operator clears the cause and cycles the selector STOP -> RUN.
    plant.motorRunTemp_c = 45.0f;
    io.runSwitch = false;
    advance(c, plant, now, 500);
    CHECK(c.state() == State::STOP_ESTOP);
    CHECK(c.fault() == Fault::NONE);

    io.runSwitch = true;
    REQUIRE(advanceUntil(c, plant, now, isRunning, 60000));
    CHECK(io.pump);
}
