// EQSP32.h — host fake of the EQSP32 library, for testing firmware.ino natively.
//
// Implements the handful of EQSP32 methods the sketch uses (begin, pinMode,
// configTIN, configPCC, readPin, pinValue) plus the mode/trigger enum names.
// readPin/pinValue dispatch on the channel number to plain fields, so tests can
// script sensor inputs and read back actuator/LED outputs. Helpers set inputs in
// engineering units.
//
// NOTE: the pin numbers below MUST match the PIN_* map in firmware/firmware.ino
// (a single-file sketch keeps its own copy of the channel map).
#pragma once

#include <cmath>

// EQSP32 pin-mode and trigger tokens used by the sketch (values are arbitrary).
enum EqMode { DIN, AIN, CIN, PCC, POUT, TIN, RELAY, SWT };
enum EqTrig { STATE, ON_RISING, ON_FALLING, ON_TOGGLE };

class EQSP32 {
public:
    // Pin map (must mirror firmware.ino).
    enum {
        P_PRESSURE_IN = 1, P_PRESSURE_OUT = 2, P_CLEAN_TANK = 3, P_SUPPLY_TANK = 4,
        P_TDS_IN = 5, P_TDS_OUT = 6, P_RUN_STOP = 7, P_MOTOR_NTC = 8,
        P_FLOW_IN = 9, P_FLOW_OUT = 10, P_LED_R = 11, P_LED_G = 12, P_LED_B = 13,
        P_FLUSH_VALVE = 14, P_PUMP = 15, P_SUPPLY_VALVE = 16,
    };

    // --- Inputs (raw EQSP32 readPin units) --- defaults = healthy at rest -----
    int pressIn_raw = 400, pressOut_raw = 400, cleanTank_raw = 400, supplyTank_raw = 400;
    int tdsIn_mv = 1500, tdsOut_mv = 250, motorTemp_raw = 250;
    bool runSwitch = true;
    int feedPulses = 0, permeatePulses = 0;

    // --- Decoded outputs (set via pinValue) ----------------------------------
    bool pump = false, supply = false, flush = false;
    bool ledR = false, ledG = false, ledB = false;

    // --- EQSP32 API used by the sketch ---------------------------------------
    void begin(bool = false) {}
    void pinMode(int, int, int = 500) {}
    void configTIN(int, int, int) {}
    void configPCC(int, int) {}

    int readPin(int pin, int = STATE) {
        switch (pin) {
            case P_PRESSURE_IN:  return pressIn_raw;
            case P_PRESSURE_OUT: return pressOut_raw;
            case P_CLEAN_TANK:   return cleanTank_raw;
            case P_SUPPLY_TANK:  return supplyTank_raw;
            case P_TDS_IN:       return tdsIn_mv;
            case P_TDS_OUT:      return tdsOut_mv;
            case P_MOTOR_NTC:    return motorTemp_raw;
            case P_RUN_STOP:     return runSwitch ? 1 : 0;
            case P_FLOW_IN:  { int p = feedPulses;     feedPulses = 0;     return p; }
            case P_FLOW_OUT: { int p = permeatePulses; permeatePulses = 0; return p; }
            default: return 0;
        }
    }
    bool pinValue(int pin, int v) {
        switch (pin) {
            case P_PUMP:         pump   = (v != 0); break;
            case P_SUPPLY_VALVE: supply = (v != 0); break;
            case P_FLUSH_VALVE:  flush  = (v != 0); break;
            case P_LED_R: ledR = (v != 0); break;
            case P_LED_G: ledG = (v != 0); break;
            case P_LED_B: ledB = (v != 0); break;
            default: break;
        }
        return true;
    }

    // --- Test helpers: set inputs in engineering units -----------------------
    static int rnd(float v) { return (int)std::lround(v); }
    void setPressureInPsi(float psi)  { pressIn_raw  = 400 + rnd(psi * (1600.0f / 300.0f)); }
    void setPressureOutPsi(float psi) { pressOut_raw = 400 + rnd(psi * (1600.0f / 300.0f)); }
    void setCleanCm(float cm)         { cleanTank_raw  = 400 + rnd(cm * (1600.0f / 500.0f)); }
    void setSupplyCm(float cm)        { supplyTank_raw = 400 + rnd(cm * (1600.0f / 500.0f)); }
    void setTdsInMgL(float mgL)       { tdsIn_mv  = rnd(mgL * (5000.0f / 10000.0f)); }
    void setTdsOutMgL(float mgL)      { tdsOut_mv = rnd(mgL * (5000.0f / 1000.0f)); }
    void setMotorC(float c)           { motorTemp_raw = rnd(c * 10.0f); }
};
