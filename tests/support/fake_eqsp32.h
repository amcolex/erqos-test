// fake_eqsp32.h — an in-memory stand-in for the EQSP32, for host tests.
//
// Implements the same two verbs the controller uses (readPin / pinValue) by
// dispatching on the spec §3 channel map to plain fields. Inputs are settable as
// raw EQSP32 values (so tests can inject broken-wire readings) and via
// engineering-unit helpers. Outputs are decoded into bool fields for assertions.
#pragma once

#include <cmath>
#include "eqsp32_port.h"
#include "ro_types.h"

namespace ro {

struct FakeEqsp32 : IEqsp32 {
    // --- Raw inputs (EQSP32 readPin units) --- defaults = healthy at rest -----
    int pressIn_raw   = 400;   // 4 mA  = 0 PSI
    int pressOut_raw  = 400;
    int cleanTank_raw = 400;   // 4 mA  = 0 cm
    int supplyTank_raw= 400;
    int tdsIn_mv  = 1500;      // ~3000 mg/L feed
    int tdsOut_mv = 250;       // ~50 mg/L permeate
    int motorTemp_raw = 250;   // 25.0 C
    bool runSwitch = true;     // RUN
    int feedPulses = 0;        // pending pulses, returned then cleared
    int permeatePulses = 0;

    // --- Decoded outputs (set by pinValue) -----------------------------------
    bool pump = false, supply = false, flush = false;
    Rgb  rgb{};

    // --- IEqsp32 -------------------------------------------------------------
    int readPin(int pin) override {
        switch (pin) {
            case ch::PRESSURE_IN:  return pressIn_raw;
            case ch::PRESSURE_OUT: return pressOut_raw;
            case ch::CLEAN_TANK:   return cleanTank_raw;
            case ch::SUPPLY_TANK:  return supplyTank_raw;
            case ch::TDS_IN:       return tdsIn_mv;
            case ch::TDS_OUT:      return tdsOut_mv;
            case ch::MOTOR_NTC:    return motorTemp_raw;
            case ch::RUN_STOP:     return runSwitch ? 1 : 0;
            case ch::FLOW_IN:  { int p = feedPulses;     feedPulses = 0;     return p; }
            case ch::FLOW_OUT: { int p = permeatePulses; permeatePulses = 0; return p; }
            default: return 0;
        }
    }
    void pinValue(int pin, int v) override {
        switch (pin) {
            case ch::PUMP:         pump   = (v != 0); break;
            case ch::SUPPLY_VALVE: supply = (v != 0); break;
            case ch::FLUSH_VALVE:  flush  = (v != 0); break;
            case ch::LED_R: rgb.r = (v != 0); break;
            case ch::LED_G: rgb.g = (v != 0); break;
            case ch::LED_B: rgb.b = (v != 0); break;
            default: break;
        }
    }

    // --- Engineering-unit helpers (inverse of sensors.h scaling) -------------
    static int rnd(float v) { return (int)std::lround(v); }
    void setPressureInPsi(float psi)  { pressIn_raw  = 400 + rnd(psi * (1600.0f / 300.0f)); }
    void setPressureOutPsi(float psi) { pressOut_raw = 400 + rnd(psi * (1600.0f / 300.0f)); }
    void setCleanCm(float cm)         { cleanTank_raw  = 400 + rnd(cm * (1600.0f / 500.0f)); }
    void setSupplyCm(float cm)        { supplyTank_raw = 400 + rnd(cm * (1600.0f / 500.0f)); }
    void setTdsInMgL(float mgL)       { tdsIn_mv  = rnd(mgL * (5000.0f / 10000.0f)); }
    void setTdsOutMgL(float mgL)      { tdsOut_mv = rnd(mgL * (5000.0f / 1000.0f)); }
    void setMotorC(float c)           { motorTemp_raw = rnd(c * 10.0f); }
};

} // namespace ro
