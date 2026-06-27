// plant_model.h — a virtual RO skid for closed-loop scenario testing.
//
// Each step() reads the actuator commands the controller wrote to FakeEqsp32 and
// evolves the process variables (pressure, flow, permeate, clean-tank level,
// motor temperature, flow-meter pulses) over dt, writing them back as sensor
// readings. This lets scenario tests exercise the controller exactly as it would
// run against real plant dynamics, but in microseconds of wall-clock time.
//
// Fields are public on purpose: tests tweak them to create conditions
// (fouling, dry feed, supply low, bad permeate, hot motor, broken sensor).
#pragma once

#include "support/fake_eqsp32.h"

namespace ro {

class PlantModel {
public:
    explicit PlantModel(FakeEqsp32& io) : io_(io) {}

    // --- Process state (engineering units) ---------------------------------
    float pressIn_psi   = 0.0f;
    float pressOut_psi  = 0.0f;
    float feedFlow_lpm  = 0.0f;
    float permeateFlow_lpm = 0.0f;
    float cleanTank_cm  = 100.0f;   // product tank starts part-full
    float supplyTank_cm = 200.0f;   // ample feed by default (test-controlled)
    float motorTemp_c   = 25.0f;
    float feedTds_mgL   = 3000.0f;
    float permeateTds_mgL = 50.0f;

    // --- Behaviour knobs (defaults model a healthy skid) -------------------
    float feedNominal_lpm   = 10.0f;  // feed flow when pump+supply on
    float producePress_psi  = 175.0f; // membrane pressure while producing
    float flushPress_psi    = 30.0f;  // pressure while flushing (restrictor bypassed)
    float membraneDp_psi    = 10.0f;  // pressIn - pressOut
    float recoveryFrac      = 0.20f;  // permeate / feed while producing
    float foulingOffset_psi = 0.0f;   // add to producing pressure (fouling/over-pressure)
    float motorRunTemp_c    = 45.0f;  // steady motor temp while running
    float motorAmbient_c    = 25.0f;
    float cleanLitersPerCm  = 0.2f;   // product tank cross-section
    float cleanDraw_lpm     = 1.0f;   // downstream consumption (drains the tank)
    bool  blockFeed         = false;  // simulate no feed water (dry-run)
    bool  breakPressInWire  = false;  // simulate a broken 4-20 mA loop on pressure-IN

    // Time constants (ms) for first-order responses.
    uint32_t flowTau_ms  = 1500;
    uint32_t pressTau_ms = 1500;
    uint32_t tempTau_ms  = 20000;

    void step(uint32_t dt_ms);

private:
    FakeEqsp32& io_;
    float pulseCarryFeed_ = 0.0f;
    float pulseCarryPerm_ = 0.0f;
    static float approach(float cur, float target, uint32_t dt_ms, uint32_t tau_ms);
};

} // namespace ro
