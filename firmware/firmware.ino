/**
 * @file    firmware.ino
 * @brief   Reverse-Osmosis skid controller for the Erqos EQSP32.
 *
 * Thin Arduino entry point. All control logic lives in the platform-independent
 * core under src/ (ro_controller, sensors, rgb_indicator) and is exercised by
 * the host test suite in ../tests. This file only wires the EQSP32-backed I/O
 * adapter to the controller and pumps the control loop.
 *
 * Build / flash / monitor:
 *   pixi run build      pixi run flash      pixi run monitor   (115200 baud)
 */

#include "EQSP32.h"
#include "src/eqsp32_io.h"
#include "src/ro_controller.h"

EQSP32 eqsp32;
ro::Eqsp32SkidIO skidIo(eqsp32);
ro::RoController controller(skidIo);   // default Params = spec §11 setpoints

// Control loop period. The EQSP32 runs its own services on the second core; we
// just sample, decide and actuate without blocking.
static constexpr uint32_t LOOP_PERIOD_MS = 20;

static uint32_t lastReport = 0;

static const char* stateName(ro::State s) {
    using ro::State;
    switch (s) {
        case State::STOP_ESTOP:        return "STOP";
        case State::STOPPED_TANK_FULL: return "TANK_FULL";
        case State::PAUSED_SUPPLY_LOW: return "SUPPLY_LOW";
        case State::STARTING:          return "STARTING";
        case State::RUNNING:           return "RUNNING";
        case State::FLUSHING:          return "FLUSHING";
        case State::FAULT:             return "FAULT";
    }
    return "?";
}

void setup() {
    eqsp32.begin(true);          // verbose boot diagnostics
    Serial.begin(115200);
    skidIo.begin();              // configure pin modes (spec §3)
    controller.reset(millis());
}

void loop() {
    const uint32_t now = millis();
    controller.tick(now);

    // One-line status once per second over the USB serial link.
    if (now - lastReport >= 1000) {
        lastReport = now;
        const ro::Readings& r = controller.readings();
        Serial.printf(
            "%-10s fault=%d  Pin=%.0f Pout=%.0f PSI  clean=%.0f sup=%.0f cm  "
            "feed=%.1f perm=%.1f L/min  rec=%.0f%%  TDS=%.0f  T=%.1fC  "
            "pump=%d sup=%d flush=%d  rgb=%d%d%d\n",
            stateName(controller.state()), (int)controller.fault(),
            r.pressIn_psi, r.pressOut_psi, r.cleanTank_cm, r.supplyTank_cm,
            r.feedFlow_lpm, r.permeateFlow_lpm, r.recovery_pct,
            r.tdsOut_mgL, r.motorTemp_c,
            controller.pumpCmd(), controller.supplyValveCmd(), controller.flushValveCmd(),
            controller.rgb().r, controller.rgb().g, controller.rgb().b);
    }

    delay(LOOP_PERIOD_MS);
}
