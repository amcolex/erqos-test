// plant_model.h — a virtual RO skid for closed-loop scenario testing.
//
// Each step() reads the actuator commands the sketch wrote to the fake EQSP32 and
// evolves the process variables (pressure, flow, permeate, clean-tank level,
// motor temperature, flow-meter pulses) over dt, writing them back as sensor
// readings. This drives the controller through realistic plant dynamics in
// microseconds of wall-clock time.
//
// Fields are public on purpose: tests tweak them to create conditions
// (fouling, dry feed, supply low, bad permeate, hot motor, broken sensor).
#pragma once

#include <cstdint>
#include "EQSP32.h"   // the fake device the sketch wrote to

class PlantModel {
public:
    explicit PlantModel(EQSP32& io) : io_(io) {}

    // --- Process state (engineering units) ---------------------------------
    float pressIn_psi   = 0.0f;
    float pressOut_psi  = 0.0f;
    float feedFlow_lpm  = 0.0f;
    float permeateFlow_lpm = 0.0f;
    float cleanTank_cm  = 100.0f;
    float supplyTank_cm = 200.0f;
    float motorTemp_c   = 25.0f;
    float feedTds_mgL   = 3000.0f;
    float permeateTds_mgL = 50.0f;

    // --- Behaviour knobs (defaults model a healthy skid) -------------------
    float feedNominal_lpm   = 10.0f;
    float producePress_psi  = 175.0f;
    float flushPress_psi    = 30.0f;
    float membraneDp_psi    = 10.0f;
    float recoveryFrac      = 0.20f;
    float foulingOffset_psi = 0.0f;
    float motorRunTemp_c    = 45.0f;
    float motorAmbient_c    = 25.0f;
    float cleanLitersPerCm  = 0.2f;
    float cleanDraw_lpm     = 1.0f;
    bool  blockFeed         = false;
    bool  breakPressInWire  = false;
    bool  frozen            = false;  // when true, step() leaves sensors untouched
                                      // (for tests that script inputs directly)

    uint32_t flowTau_ms  = 1500;
    uint32_t pressTau_ms = 1500;
    uint32_t tempTau_ms  = 20000;

    void step(uint32_t dt_ms);

private:
    EQSP32& io_;
    float pulseCarryFeed_ = 0.0f;
    float pulseCarryPerm_ = 0.0f;
    static float approach(float cur, float target, uint32_t dt_ms, uint32_t tau_ms);
};
