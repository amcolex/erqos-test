// sensors.h — pure sensor scaling and the flow-rate integrator (spec §4).
//
// All functions are free of side effects and hardware dependencies, so they are
// trivially unit-testable. Inputs are EQSP32 readPin() raw values.
#pragma once

#include <cstdint>

namespace ro {

// EQSP32 raw-value constants mirrored here so the core needs no EQSP32 header.
constexpr int CIN_OVER_CURRENT = -1;     // EQSP32 CIN_OC_ERROR (>21 mA)
constexpr int TIN_OPEN  = -9999;         // EQSP32 TIN_OPEN_CIRCUIT
constexpr int TIN_SHORT = 9999;          // EQSP32 TIN_SHORT_CIRCUIT

// --- 4-20 mA current-loop sensors (CIN, raw = mA x 100) --------------------
inline float mA_from_cin(int raw) { return raw / 100.0f; }

// Broken wire / over-current detection: a 4-20 mA loop reading below ~3.5 mA
// (or the over-current sentinel) indicates a sensor/wiring fault (spec §7/§10).
inline bool cinFaulted(int raw, float brokenWire_mA = 3.5f) {
    return raw <= CIN_OVER_CURRENT || raw < (int)(brokenWire_mA * 100.0f);
}

// Pressure transmitter: 4 mA = 0 PSI, 20 mA = 300 PSI (spec §4, 0-300 range).
inline float psi_from_cin(int raw) { return (raw - 400) * (300.0f / 1600.0f); }

// Depth transmitter: 4 mA = 0 cm, 20 mA = 500 cm (spec §4).
inline float cm_from_cin(int raw) { return (raw - 400) * (500.0f / 1600.0f); }

// --- 0-5 V TDS sensors (AIN, raw = mV) -------------------------------------
// Feed TDS: 0 V = 0, 5 V = 10,000 mg/L (spec §4).
inline float tdsIn_from_ain(int mv) { return mv * (10000.0f / 5000.0f); }
// Permeate TDS: 0 V = 0, 5 V = 1,000 mg/L (spec §4).
inline float tdsOut_from_ain(int mv) { return mv * (1000.0f / 5000.0f); }

// --- 10K NTC (TIN, raw = degC x 10) ----------------------------------------
inline float celsius_from_tin(int raw) { return raw / 10.0f; }
inline bool  tinValid(int raw) { return raw != TIN_OPEN && raw != TIN_SHORT; }

// ---------------------------------------------------------------------------
// FlowRate — integrates discrete pulses into L/min over a fixed time window
// (spec §4: 100 pulses = 1 L). A windowed average (vs. an instantaneous count)
// gives a stable, deterministic reading from a coarse tick rate. The value is
// refreshed once per window and held in between.
// ---------------------------------------------------------------------------
class FlowRate {
public:
    explicit FlowRate(float pulsesPerLiter = 100.0f, uint32_t windowMs = 1000)
        : pulsesPerLiter_(pulsesPerLiter), windowMs_(windowMs) {}

    // Feed the pulses returned this tick and the current time. Returns L/min.
    float update(int pulses, uint32_t now_ms);
    float lpm() const { return lpm_; }
    void  reset();

private:
    float    pulsesPerLiter_;
    uint32_t windowMs_;
    uint32_t winStart_ = 0;
    long     pulseAcc_ = 0;
    float    lpm_ = 0.0f;
    bool     primed_ = false;
};

} // namespace ro
