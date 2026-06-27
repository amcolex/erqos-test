/**
 * @file    firmware.ino
 * @brief   Reverse-Osmosis skid controller for the Erqos EQSP32.
 *
 * Single-file Arduino sketch implementing RO-Skid-Control-Specification.docx:
 * an 8-state machine with safety interlocks, latched faults, timed sequences,
 * level/pressure hysteresis, membrane-fouling detection and an RGB status LED.
 *
 * The whole program lives here in the classic sketch shape (config, helpers,
 * setup(), loop()). It is verified by the host test suite in ../tests, which
 * compiles THIS file against a fake EQSP32 + fake Arduino and drives setup()/
 * loop() with a virtual clock and a simulated plant — no hardware needed.
 *
 * Build / flash / monitor:
 *   pixi run build   pixi run test   pixi run flash   pixi run monitor (115200)
 */

#include "EQSP32.h"

// ============================================================================
//  Configuration
// ============================================================================

// --- Pin / channel map (spec §3) -------------------------------------------
constexpr int PIN_PRESSURE_IN  = 1;   // CIN  4-20 mA  membrane feed pressure
constexpr int PIN_PRESSURE_OUT = 2;   // CIN  4-20 mA  concentrate (brine) pressure
constexpr int PIN_CLEAN_TANK   = 3;   // CIN  4-20 mA  product tank level
constexpr int PIN_SUPPLY_TANK  = 4;   // CIN  4-20 mA  feed tank level
constexpr int PIN_TDS_IN       = 5;   // AIN  0-5 V    feed TDS
constexpr int PIN_TDS_OUT      = 6;   // AIN  0-5 V    permeate TDS
constexpr int PIN_RUN_STOP     = 7;   // DIN          run / stop selector
constexpr int PIN_MOTOR_NTC    = 8;   // TIN          pump motor temperature
constexpr int PIN_FLOW_IN      = 9;   // PCC          feed flow
constexpr int PIN_FLOW_OUT     = 10;  // PCC          permeate flow
constexpr int PIN_LED_R        = 11;  // POUT         status RGB - red
constexpr int PIN_LED_G        = 12;  // POUT         status RGB - green
constexpr int PIN_LED_B        = 13;  // POUT         status RGB - blue
constexpr int PIN_FLUSH_VALVE  = 14;  // POUT 24 V    brine-bypass / forward-flush
constexpr int PIN_PUMP         = 15;  // POUT 24 V    RO pump contactor
constexpr int PIN_SUPPLY_VALVE = 16;  // POUT 24 V    feed inlet valve

constexpr int OUT_ON  = 1000;   // POUT 0-1000 = 0-100% duty
constexpr int OUT_OFF = 0;

// --- Setpoints (spec §11) --------------------------------------------------
constexpr float SUPPLY_MIN_CM     = 50.0f;   // below -> pause
constexpr float SUPPLY_RESUME_CM  = 60.0f;   // resume at/above
constexpr float CLEAN_FULL_CM     = 400.0f;  // stop production at/above
constexpr float CLEAN_RESTART_CM  = 380.0f;  // restart below

constexpr float PRESS_BAND_LO_PSI = 150.0f;
constexpr float PRESS_BAND_HI_PSI = 200.0f;  // above = fouling indication
constexpr float OVERPRESSURE_PSI  = 250.0f;  // safety trip

constexpr float TDS_LIMIT_MGL = 200.0f;
constexpr float TDS_WARN_MGL  = 180.0f;            // "rising toward 200"
constexpr uint32_t TDS_FAULT_DELAY_MS = 10u * 60u * 1000u;

constexpr float FEED_FLOW_LO_LPM = 8.0f;
constexpr float FEED_FLOW_HI_LPM = 12.0f;
constexpr float DRY_RUN_FLOW_LPM = 0.5f;           // "~0" (spec §13 open point)
constexpr uint32_t DRY_RUN_GRACE_MS = 10u * 1000u;
constexpr float LOW_PERMEATE_LPM = 1.0f;           // "low permeate" for cleaning-required

constexpr float RECOVERY_LO_PCT = 10.0f;
constexpr float RECOVERY_HI_PCT = 30.0f;

constexpr float MOTOR_TEMP_LIMIT_C = 80.0f;

constexpr uint32_t FLUSH_INTERVAL_MS = 60u * 60u * 1000u;  // continuous run -> flush
constexpr uint32_t FLUSH_DURATION_MS = 5u * 60u * 1000u;
constexpr uint32_t MIN_ON_MS    = 10u * 1000u;     // before a non-safety stop
constexpr uint32_t MIN_OFF_MS   = 10u * 1000u;     // before a restart
constexpr uint32_t SETTLE_MS    = 25u * 1000u;     // production alarms masked

constexpr float    BROKEN_WIRE_MA   = 3.5f;        // 4-20 mA below this = broken wire
constexpr uint32_t WARN_DEBOUNCE_MS = 3000;        // nuisance suppression

// --- RGB LED timing (spec §9/§10) ------------------------------------------
constexpr uint32_t BLUE_BLINK_MS  = 1000;   // supply-low
constexpr uint32_t WARN_SLOW_MS   = 1000;   // pressure / early fouling
constexpr uint32_t WARN_MED_MS    = 500;    // flow / recovery
constexpr uint32_t WARN_FAST_MS   = 250;    // TDS rising / cleaning required
constexpr uint32_t FAULT_ON_MS    = 200;    // each fault-code blink: on time
constexpr uint32_t FAULT_OFF_MS   = 200;    // each fault-code blink: off time
constexpr uint32_t FAULT_GAP_MS   = 1000;   // pause between code repetitions

constexpr uint32_t LOOP_PERIOD_MS = 20;

// EQSP32 readPin sentinels mirrored locally.
constexpr int CIN_OVER_CURRENT = -1;     // > 21 mA
constexpr int TIN_OPEN  = -9999;
constexpr int TIN_SHORT = 9999;

// ============================================================================
//  Types
// ============================================================================
enum class State : uint8_t {
    STOP_ESTOP,         // selector in STOP (or no power)
    STOPPED_TANK_FULL,  // clean tank full
    PAUSED_SUPPLY_LOW,  // supply tank low
    STARTING,           // pump on, pressure/flow settling
    RUNNING,            // producing
    FLUSHING,           // periodic forward-flush
    FAULT,              // latched error
};

// Numeric value = the red blink-code (spec §10).
enum class Fault : uint8_t {
    NONE = 0, MOTOR_OVERTEMP = 1, DRY_RUN = 2, PERMEATE_TDS = 3,
    OVERPRESSURE = 4, SENSOR = 5,
};

struct Warnings {
    bool pressure = false;    // out of band / early fouling -> slow blink
    bool flowOrRec = false;   // flow or recovery out of band -> medium blink
    bool tdsOrClean = false;  // TDS rising / cleaning required -> fast blink
    bool any() const { return pressure || flowOrRec || tdsOrClean; }
};

// Plain aggregate (no default member initialisers) so brace-init like
// Rgb{true,false,false} works under the ESP32 core's C++11.
struct Rgb { bool r, g, b; };

// Asserts only after a condition has held continuously for `ms` (wrap-safe).
struct Debouncer {
    uint32_t since = 0; bool counting = false, latched = false;
    bool update(bool cond, uint32_t now, uint32_t ms) {
        if (!cond) { counting = false; latched = false; return false; }
        if (!counting) { counting = true; since = now; }
        if ((uint32_t)(now - since) >= ms) latched = true;
        return latched;
    }
    void reset() { counting = false; latched = false; since = 0; }
};

// Integrates discrete pulses (100 pulses = 1 L, spec §4) into L/min over a
// fixed window; value is refreshed once per window and held in between.
struct FlowRate {
    float pulsesPerLiter = 100.0f; uint32_t windowMs = 1000;
    uint32_t winStart = 0; long acc = 0; float lpm = 0; bool primed = false;
    float update(int pulses, uint32_t now) {
        if (!primed) { winStart = now; acc = 0; primed = true; }
        acc += pulses;
        uint32_t dt = now - winStart;
        if (dt >= windowMs && dt > 0) {
            lpm = (acc / pulsesPerLiter) / (dt / 60000.0f);
            winStart = now; acc = 0;
        }
        return lpm;
    }
};

// ============================================================================
//  Pure helpers — sensor scaling (spec §4) and RGB resolution (spec §9/§10)
// ============================================================================
static float psiFromCin(int raw)  { return (raw - 400) * (300.0f / 1600.0f); }   // 4 mA=0, 20 mA=300
static float cmFromCin(int raw)   { return (raw - 400) * (500.0f / 1600.0f); }   // 4 mA=0, 20 mA=500
static float tdsInFromAin(int mv) { return mv * (10000.0f / 5000.0f); }          // 0-5 V = 0-10000
static float tdsOutFromAin(int mv){ return mv * (1000.0f / 5000.0f); }           // 0-5 V = 0-1000
static float celsiusFromTin(int raw) { return raw / 10.0f; }
static bool  tinValid(int raw)    { return raw != TIN_OPEN && raw != TIN_SHORT; }
static bool  cinFaulted(int raw)  { return raw <= CIN_OVER_CURRENT || raw < (int)(BROKEN_WIRE_MA * 100); }

static bool blink5050(uint32_t now, uint32_t period) {
    return period == 0 ? true : (now % period) < (period / 2);
}
static bool faultCodeOn(Fault f, uint32_t now) {
    int code = (int)f;
    if (code <= 0) return false;
    uint32_t unit = FAULT_ON_MS + FAULT_OFF_MS;
    uint32_t cycle = code * unit + FAULT_GAP_MS;
    uint32_t phase = now % cycle;
    if (phase >= (uint32_t)code * unit) return false;     // in the gap
    return (phase % unit) < FAULT_ON_MS;
}

// ============================================================================
//  Runtime state — all mutable state lives here so setup() can reset it in one
//  line. The only other globals are `eqsp32` and the millisecond clock.
// ============================================================================
struct AppState {
    State state = State::STOP_ESTOP;
    Fault fault = Fault::NONE;
    Warnings warn;
    Rgb rgb;

    // latest readings (engineering units) — for the status report
    float pressIn_psi = 0, pressOut_psi = 0, cleanTank_cm = 0, supplyTank_cm = 0;
    float tdsOut_mgL = 0, motorTemp_c = 0, feedFlow_lpm = 0, permeateFlow_lpm = 0, recovery_pct = 0;
    bool sensorValid = true, runSwitch = false;

    bool pump = false, supply = false, flush = false, prevPump = false;
    uint32_t stateSince = 0, pumpOnSince = 0, pumpOffSince = 0, runSince = 0, lastReport = 0;

    Debouncer dryRun, tdsFaultT, pressWarn, flowWarn, tdsWarn;
    FlowRate feedFlow, permeateFlow;
};

EQSP32 eqsp32;
AppState S;

// ============================================================================
//  Control logic
// ============================================================================
static void enterState(State s, uint32_t now) {
    S.state = s;
    S.stateSince = now;
    if (s == State::RUNNING) S.runSince = now;   // (re)start the flush interval
}
static void latchFault(Fault f, uint32_t now) { S.fault = f; enterState(State::FAULT, now); }

static void readInputs(uint32_t now) {
    int pIn = eqsp32.readPin(PIN_PRESSURE_IN);
    int pOut = eqsp32.readPin(PIN_PRESSURE_OUT);
    int clean = eqsp32.readPin(PIN_CLEAN_TANK);
    int supply = eqsp32.readPin(PIN_SUPPLY_TANK);
    int ntc = eqsp32.readPin(PIN_MOTOR_NTC);

    S.pressIn_psi   = psiFromCin(pIn);
    S.pressOut_psi  = psiFromCin(pOut);
    S.cleanTank_cm  = cmFromCin(clean);
    S.supplyTank_cm = cmFromCin(supply);
    S.tdsOut_mgL    = tdsOutFromAin(eqsp32.readPin(PIN_TDS_OUT));
    S.motorTemp_c   = celsiusFromTin(ntc);

    S.feedFlow_lpm     = S.feedFlow.update(eqsp32.readPin(PIN_FLOW_IN), now);
    S.permeateFlow_lpm = S.permeateFlow.update(eqsp32.readPin(PIN_FLOW_OUT), now);
    S.recovery_pct = (S.feedFlow_lpm > 0.01f) ? (S.permeateFlow_lpm / S.feedFlow_lpm) * 100.0f : 0.0f;

    S.runSwitch = eqsp32.readPin(PIN_RUN_STOP) != 0;   // contact closed = RUN
    S.sensorValid = !cinFaulted(pIn) && !cinFaulted(pOut) && !cinFaulted(clean) &&
                    !cinFaulted(supply) && tinValid(ntc);
}

// Always-active safety faults (spec §6.4: active even during settling).
static Fault detectSafetyFault(uint32_t now) {
    if (!S.sensorValid) { S.dryRun.reset(); return Fault::SENSOR; }
    if (S.motorTemp_c >= MOTOR_TEMP_LIMIT_C) { S.dryRun.reset(); return Fault::MOTOR_OVERTEMP; }
    if (S.pressIn_psi > OVERPRESSURE_PSI || S.pressOut_psi > OVERPRESSURE_PSI) {
        S.dryRun.reset(); return Fault::OVERPRESSURE;
    }
    bool dryCond = S.prevPump && (S.feedFlow_lpm < DRY_RUN_FLOW_LPM);
    if (S.dryRun.update(dryCond, now, DRY_RUN_GRACE_MS)) return Fault::DRY_RUN;
    return Fault::NONE;
}

static bool permissivesMet(uint32_t now) {
    return S.runSwitch && S.fault == Fault::NONE &&
           S.supplyTank_cm >= SUPPLY_MIN_CM &&
           S.cleanTank_cm  <  CLEAN_RESTART_CM &&
           S.motorTemp_c   <  MOTOR_TEMP_LIMIT_C &&
           (uint32_t)(now - S.pumpOffSince) >= MIN_OFF_MS;
}

static void startOrHold(uint32_t now) {
    if (permissivesMet(now)) { enterState(State::STARTING, now); return; }
    if (S.cleanTank_cm >= CLEAN_RESTART_CM) { enterState(State::STOPPED_TANK_FULL, now); return; }
    if (S.supplyTank_cm < SUPPLY_MIN_CM)    { enterState(State::PAUSED_SUPPLY_LOW, now); return; }
    if (S.state != State::STOPPED_TANK_FULL && S.state != State::PAUSED_SUPPLY_LOW)
        enterState(State::STOPPED_TANK_FULL, now);   // waiting on min OFF-time
}

static void runStateMachine(uint32_t now) {
    const bool minOnElapsed = (uint32_t)(now - S.pumpOnSince) >= MIN_ON_MS;
    switch (S.state) {
        case State::STOP_ESTOP:
            startOrHold(now);
            break;
        case State::STOPPED_TANK_FULL:                       // spec §6.7 hysteresis
            if (S.cleanTank_cm < CLEAN_RESTART_CM) startOrHold(now);
            break;
        case State::PAUSED_SUPPLY_LOW:                       // spec §6.8 hysteresis
            if (S.supplyTank_cm >= SUPPLY_RESUME_CM) startOrHold(now);
            break;
        case State::STARTING:                                // spec §6.4
            if ((uint32_t)(now - S.stateSince) >= SETTLE_MS) enterState(State::RUNNING, now);
            break;
        case State::RUNNING:
            if (minOnElapsed && S.cleanTank_cm >= CLEAN_FULL_CM) {
                enterState(State::STOPPED_TANK_FULL, now);   // spec §6.7
            } else if (minOnElapsed && S.supplyTank_cm < SUPPLY_MIN_CM) {
                enterState(State::PAUSED_SUPPLY_LOW, now);   // spec §6.8
            } else if ((uint32_t)(now - S.runSince) >= FLUSH_INTERVAL_MS) {
                enterState(State::FLUSHING, now);            // spec §6.6
            }
            break;
        case State::FLUSHING:                                // spec §6.6
            if (S.supplyTank_cm < SUPPLY_MIN_CM) enterState(State::PAUSED_SUPPLY_LOW, now);
            else if ((uint32_t)(now - S.stateSince) >= FLUSH_DURATION_MS) enterState(State::RUNNING, now);
            break;
        case State::FAULT:
            break;
    }
}

// Production warnings (spec §7/§8/§9) — only while RUNNING, so naturally masked
// during the start settling window and during a flush.
static void updateWarnings(uint32_t now) {
    if (S.state != State::RUNNING) {
        S.warn = {};
        S.pressWarn.reset(); S.flowWarn.reset(); S.tdsWarn.reset();
        S.tdsFaultT.reset();   // don't carry a TDS countdown across a stop/restart
        return;
    }
    bool pressCond = S.pressIn_psi  < PRESS_BAND_LO_PSI || S.pressIn_psi  > PRESS_BAND_HI_PSI ||
                     S.pressOut_psi < PRESS_BAND_LO_PSI || S.pressOut_psi > PRESS_BAND_HI_PSI;
    bool flowCond  = S.feedFlow_lpm < FEED_FLOW_LO_LPM || S.feedFlow_lpm > FEED_FLOW_HI_LPM ||
                     S.recovery_pct < RECOVERY_LO_PCT  || S.recovery_pct > RECOVERY_HI_PCT;
    bool cleaning  = S.pressIn_psi > PRESS_BAND_HI_PSI && S.permeateFlow_lpm < LOW_PERMEATE_LPM;
    bool tdsCond   = S.tdsOut_mgL > TDS_WARN_MGL || cleaning;

    S.warn.pressure   = S.pressWarn.update(pressCond, now, WARN_DEBOUNCE_MS);
    S.warn.flowOrRec  = S.flowWarn.update(flowCond,  now, WARN_DEBOUNCE_MS);
    S.warn.tdsOrClean = S.tdsWarn.update(tdsCond,    now, WARN_DEBOUNCE_MS);
}

static Rgb resolveRgb(uint32_t now) {
    const Rgb OFF{false,false,false}, RED{true,false,false}, GREEN{false,true,false};
    const Rgb BLUE{false,false,true}, YELLOW{true,true,false};
    switch (S.state) {
        case State::STOP_ESTOP:        return OFF;
        case State::FAULT:             return faultCodeOn(S.fault, now) ? RED : OFF;
        case State::FLUSHING:          return YELLOW;                       // solid
        case State::STARTING:          return GREEN;
        case State::RUNNING:
            if (S.warn.any()) {
                uint32_t period = S.warn.tdsOrClean ? WARN_FAST_MS
                                : S.warn.flowOrRec  ? WARN_MED_MS : WARN_SLOW_MS;
                return blink5050(now, period) ? YELLOW : OFF;
            }
            return GREEN;
        case State::PAUSED_SUPPLY_LOW: return blink5050(now, BLUE_BLINK_MS) ? BLUE : OFF;
        case State::STOPPED_TANK_FULL: return BLUE;                         // solid
    }
    return OFF;
}

static void applyOutputs(uint32_t now) {
    switch (S.state) {
        case State::STARTING:
        case State::RUNNING:  S.pump = true;  S.supply = true;  S.flush = false; break;
        case State::FLUSHING: S.pump = true;  S.supply = true;  S.flush = true;  break;
        default:              S.pump = false; S.supply = false; S.flush = false; break;
    }
    if (S.pump && !S.prevPump) S.pumpOnSince = now;
    if (!S.pump && S.prevPump) S.pumpOffSince = now;
    S.prevPump = S.pump;

    eqsp32.pinValue(PIN_PUMP,         S.pump   ? OUT_ON : OUT_OFF);
    eqsp32.pinValue(PIN_SUPPLY_VALVE, S.supply ? OUT_ON : OUT_OFF);
    eqsp32.pinValue(PIN_FLUSH_VALVE,  S.flush  ? OUT_ON : OUT_OFF);

    S.rgb = resolveRgb(now);
    eqsp32.pinValue(PIN_LED_R, S.rgb.r ? OUT_ON : OUT_OFF);
    eqsp32.pinValue(PIN_LED_G, S.rgb.g ? OUT_ON : OUT_OFF);
    eqsp32.pinValue(PIN_LED_B, S.rgb.b ? OUT_ON : OUT_OFF);
}

static const char* stateName() {
    switch (S.state) {
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

static void reportStatus(uint32_t now) {
    if (now - S.lastReport < 1000) return;
    S.lastReport = now;
    Serial.printf("%-10s fault=%d  Pin=%.0f Pout=%.0f PSI  clean=%.0f sup=%.0f cm  "
                  "feed=%.1f perm=%.1f L/min  rec=%.0f%%  TDS=%.0f  T=%.1fC  "
                  "pump=%d sup=%d flush=%d  rgb=%d%d%d\n",
                  stateName(), (int)S.fault, S.pressIn_psi, S.pressOut_psi,
                  S.cleanTank_cm, S.supplyTank_cm, S.feedFlow_lpm, S.permeateFlow_lpm,
                  S.recovery_pct, S.tdsOut_mgL, S.motorTemp_c,
                  S.pump, S.supply, S.flush, S.rgb.r, S.rgb.g, S.rgb.b);
}

static void configurePins() {
    eqsp32.pinMode(PIN_PRESSURE_IN,  CIN);
    eqsp32.pinMode(PIN_PRESSURE_OUT, CIN);
    eqsp32.pinMode(PIN_CLEAN_TANK,   CIN);
    eqsp32.pinMode(PIN_SUPPLY_TANK,  CIN);
    eqsp32.pinMode(PIN_TDS_IN,  AIN);
    eqsp32.pinMode(PIN_TDS_OUT, AIN);
    eqsp32.pinMode(PIN_RUN_STOP, DIN);
    eqsp32.pinMode(PIN_MOTOR_NTC, TIN);
    eqsp32.configTIN(PIN_MOTOR_NTC, 3435, 10000);   // 10K NTC, Beta 3435 (spec §4)
    eqsp32.pinMode(PIN_FLOW_IN,  PCC);
    eqsp32.configPCC(PIN_FLOW_IN, ON_RISING);
    eqsp32.pinMode(PIN_FLOW_OUT, PCC);
    eqsp32.configPCC(PIN_FLOW_OUT, ON_RISING);
    eqsp32.pinMode(PIN_LED_R,        POUT);
    eqsp32.pinMode(PIN_LED_G,        POUT);
    eqsp32.pinMode(PIN_LED_B,        POUT);
    eqsp32.pinMode(PIN_FLUSH_VALVE,  POUT);
    eqsp32.pinMode(PIN_PUMP,         POUT);
    eqsp32.pinMode(PIN_SUPPLY_VALVE, POUT);
}

// ============================================================================
//  Arduino entry points
// ============================================================================
void setup() {
    eqsp32.begin(true);
    Serial.begin(115200);
    configurePins();

    uint32_t now = millis();
    S = AppState{};                       // reset all runtime state
    S.stateSince = now;
    S.pumpOnSince = now;
    S.pumpOffSince = now - MIN_OFF_MS;    // min OFF-time already satisfied at boot (spec §6.2)
    S.runSince = now;
}

void loop() {
    const uint32_t now = millis();
    readInputs(now);

    // Emergency stop / reset (spec §6.1): de-energise everything, clear the latch.
    if (!S.runSwitch) {
        S.fault = Fault::NONE; S.warn = {};
        S.dryRun.reset(); S.tdsFaultT.reset();
        if (S.state != State::STOP_ESTOP) enterState(State::STOP_ESTOP, now);
        applyOutputs(now);
        reportStatus(now);
        delay(LOOP_PERIOD_MS);
        return;
    }

    if (S.state == State::FAULT) {            // latched until STOP (handled above)
        applyOutputs(now);
        reportStatus(now);
        delay(LOOP_PERIOD_MS);
        return;
    }

    Fault sf = detectSafetyFault(now);        // immediate, bypasses min ON-time
    if (sf != Fault::NONE) { latchFault(sf, now); applyOutputs(now); reportStatus(now); delay(LOOP_PERIOD_MS); return; }

    // Permeate out-of-spec: production fault, only while RUNNING (spec §6.4).
    if (S.state == State::RUNNING &&
        S.tdsFaultT.update(S.tdsOut_mgL > TDS_LIMIT_MGL, now, TDS_FAULT_DELAY_MS)) {
        latchFault(Fault::PERMEATE_TDS, now); applyOutputs(now); reportStatus(now); delay(LOOP_PERIOD_MS); return;
    }

    runStateMachine(now);
    updateWarnings(now);
    applyOutputs(now);
    reportStatus(now);
    delay(LOOP_PERIOD_MS);
}
