// fake_io.h — scriptable ISkidIO for host tests.
//
// Inputs are settable as raw EQSP32 values (so tests can inject broken-wire
// readings etc.) and via engineering-unit helpers for readability. Outputs are
// recorded for assertions. Pulse counters auto-clear on read, exactly like the
// EQSP32 PCC behaviour.
#pragma once

#include <cmath>
#include "io_interface.h"
#include "ro_types.h"

namespace ro {

struct FakeSkidIO : ISkidIO {
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

    // --- Recorded outputs ----------------------------------------------------
    bool pump = false, supply = false, flush = false;
    Rgb  rgb{};

    // --- ISkidIO -------------------------------------------------------------
    int  readPressureIn()    override { return pressIn_raw; }
    int  readPressureOut()   override { return pressOut_raw; }
    int  readCleanTank()     override { return cleanTank_raw; }
    int  readSupplyTank()    override { return supplyTank_raw; }
    int  readTdsIn()         override { return tdsIn_mv; }
    int  readTdsOut()        override { return tdsOut_mv; }
    int  readMotorTemp()     override { return motorTemp_raw; }
    bool readRunSwitch()     override { return runSwitch; }
    int  readFeedPulses()    override { int p = feedPulses; feedPulses = 0; return p; }
    int  readPermeatePulses()override { int p = permeatePulses; permeatePulses = 0; return p; }

    void setPump(bool on)          override { pump = on; }
    void setSupplyValve(bool open) override { supply = open; }
    void setFlushValve(bool open)  override { flush = open; }
    void setRgb(bool r, bool g, bool b) override { rgb = Rgb(r, g, b); }

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
