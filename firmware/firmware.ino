/**
 * @file    firmware.ino
 * @brief   Reverse-Osmosis skid controller for the Erqos EQSP32.
 *
 * This is the ONLY file that includes <EQSP32.h>. Everything under src/ is
 * platform-independent control logic, exercised by the host test suite in
 * ../tests. Here we just:
 *   1. adapt the real EQSP32 to the controller's 2-verb port (Eqsp32Port),
 *   2. configure the pin modes for the spec §3 channel map,
 *   3. pump the control loop.
 *
 * Build / flash / monitor:
 *   pixi run build      pixi run flash      pixi run monitor   (115200 baud)
 */

#include "EQSP32.h"
#include "src/eqsp32_port.h"
#include "src/ro_controller.h"

// --- Production port: forward the two verbs to the real EQSP32 ---------------
struct Eqsp32Port : ro::IEqsp32 {
    EQSP32& e;
    explicit Eqsp32Port(EQSP32& eq) : e(eq) {}
    int  readPin(int pin) override        { return e.readPin(pin); }
    void pinValue(int pin, int v) override { e.pinValue(pin, v); }
};

EQSP32 eqsp32;
Eqsp32Port port(eqsp32);
ro::RoController controller(port);   // default Params = spec §11 setpoints

// Control loop period. The EQSP32 runs its own services on the second core; we
// just sample, decide and actuate without blocking.
static constexpr uint32_t LOOP_PERIOD_MS = 20;
static uint32_t lastReport = 0;

// Configure the EQSP32 pins for the spec §3 I/O map. Call after eqsp32.begin().
static void configurePins() {
    using namespace ro::ch;
    eqsp32.pinMode(PRESSURE_IN,  CIN);   // 4-20 mA current loops
    eqsp32.pinMode(PRESSURE_OUT, CIN);
    eqsp32.pinMode(CLEAN_TANK,   CIN);
    eqsp32.pinMode(SUPPLY_TANK,  CIN);
    eqsp32.pinMode(TDS_IN,  AIN);        // 0-5 V analog
    eqsp32.pinMode(TDS_OUT, AIN);
    eqsp32.pinMode(RUN_STOP, DIN);       // run/stop selector
    eqsp32.pinMode(MOTOR_NTC, TIN);      // 10K NTC, Beta 3435, 10k ref (spec §4)
    eqsp32.configTIN(MOTOR_NTC, 3435, 10000);
    eqsp32.pinMode(FLOW_IN,  PCC);       // pulse flow meters
    eqsp32.configPCC(FLOW_IN, ON_RISING);
    eqsp32.pinMode(FLOW_OUT, PCC);
    eqsp32.configPCC(FLOW_OUT, ON_RISING);
    eqsp32.pinMode(LED_R,        POUT);  // RGB + valves + pump
    eqsp32.pinMode(LED_G,        POUT);
    eqsp32.pinMode(LED_B,        POUT);
    eqsp32.pinMode(FLUSH_VALVE,  POUT);
    eqsp32.pinMode(PUMP,         POUT);
    eqsp32.pinMode(SUPPLY_VALVE, POUT);
}

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
    configurePins();             // spec §3
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
