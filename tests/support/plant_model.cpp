// plant_model.cpp — virtual RO skid dynamics.
#include "support/plant_model.h"
#include <algorithm>

namespace ro {

// First-order approach toward target with time constant tau (linearised).
float PlantModel::approach(float cur, float target, uint32_t dt_ms, uint32_t tau_ms) {
    if (tau_ms == 0) return target;
    float a = (float)dt_ms / (float)tau_ms;
    if (a > 1.0f) a = 1.0f;
    return cur + (target - cur) * a;
}

void PlantModel::step(uint32_t dt_ms) {
    const float dt_min = dt_ms / 60000.0f;

    const bool pump = io_.pump;
    const bool supplyOpen = io_.supply;
    const bool flushOpen = io_.flush;
    const bool running = pump && supplyOpen;
    const bool producing = running && !flushOpen;

    // --- Feed flow ---------------------------------------------------------
    float feedTarget = (running && !blockFeed) ? feedNominal_lpm : 0.0f;
    feedFlow_lpm = approach(feedFlow_lpm, feedTarget, dt_ms, flowTau_ms);

    // --- Pressure ----------------------------------------------------------
    float pTarget;
    if (!running)        pTarget = 0.0f;
    else if (flushOpen)  pTarget = flushPress_psi;
    else                 pTarget = producePress_psi + foulingOffset_psi;
    pressIn_psi  = approach(pressIn_psi, pTarget, dt_ms, pressTau_ms);
    pressOut_psi = approach(pressOut_psi, std::max(0.0f, pTarget - membraneDp_psi), dt_ms, pressTau_ms);

    // --- Permeate ----------------------------------------------------------
    float permTarget = producing ? feedFlow_lpm * recoveryFrac : 0.0f;
    permeateFlow_lpm = approach(permeateFlow_lpm, permTarget, dt_ms, flowTau_ms);

    // --- Clean-water tank: filled by permeate, drained by downstream draw --
    cleanTank_cm += (permeateFlow_lpm - cleanDraw_lpm) * dt_min / cleanLitersPerCm;
    cleanTank_cm = std::max(0.0f, std::min(cleanTank_cm, 500.0f));

    // supplyTank_cm is treated as an external input (set by the test); not auto-drained.
    supplyTank_cm = std::max(0.0f, std::min(supplyTank_cm, 500.0f));

    // --- Motor temperature -------------------------------------------------
    motorTemp_c = approach(motorTemp_c, pump ? motorRunTemp_c : motorAmbient_c, dt_ms, tempTau_ms);

    // --- Flow-meter pulses (100 pulses/L), with fractional carry -----------
    pulseCarryFeed_ += feedFlow_lpm * dt_min * 100.0f;
    int fp = (int)pulseCarryFeed_;
    pulseCarryFeed_ -= fp;
    io_.feedPulses += fp;

    pulseCarryPerm_ += permeateFlow_lpm * dt_min * 100.0f;
    int pp = (int)pulseCarryPerm_;
    pulseCarryPerm_ -= pp;
    io_.permeatePulses += pp;

    // --- Publish to sensor inputs -----------------------------------------
    if (breakPressInWire) io_.pressIn_raw = 0;          // broken loop (~0 mA)
    else                  io_.setPressureInPsi(pressIn_psi);
    io_.setPressureOutPsi(pressOut_psi);
    io_.setCleanCm(cleanTank_cm);
    io_.setSupplyCm(supplyTank_cm);
    io_.setMotorC(motorTemp_c);
    io_.setTdsInMgL(feedTds_mgL);
    io_.setTdsOutMgL(permeateTds_mgL);
}

} // namespace ro
