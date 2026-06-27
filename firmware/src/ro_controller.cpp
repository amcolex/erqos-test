// ro_controller.cpp — RO skid control state machine implementation.
//
// Spec references in comments use the section numbers from
// RO-Skid-Control-Specification.docx.
#include "ro_controller.h"

namespace ro {

RoController::RoController(IEqsp32& eq, const Params& p, const RgbTiming& timing)
    : eq_(eq), p_(p), timing_(timing) {}

void RoController::reset(uint32_t now_ms) {
    state_ = State::STOP_ESTOP;
    fault_ = Fault::NONE;
    warn_ = {};
    rgb_ = {};
    r_ = {};
    pump_ = supply_ = flush_ = prevPump_ = false;
    stateSince_ = now_ms;
    pumpOnSince_ = now_ms;
    // Pretend the min OFF-time has already elapsed at boot so that a power-up
    // with the selector in RUN starts "immediately" (spec §6.2).
    pumpOffSince_ = now_ms - p_.minOffTime_ms;
    runSince_ = now_ms;
    dryRun_.reset(); tdsFault_.reset();
    pressWarn_.reset(); flowWarn_.reset(); tdsWarn_.reset();
    feedFlow_.reset(); permeateFlow_.reset();
    initialized_ = true;
}

void RoController::enter(State s, uint32_t now) {
    state_ = s;
    stateSince_ = now;
    if (s == State::RUNNING) runSince_ = now;  // (re)start the flush interval
}

void RoController::latch(Fault f, uint32_t now) {
    fault_ = f;
    enter(State::FAULT, now);
}

// ---------------------------------------------------------------------------
// Sensor acquisition + scaling.
// ---------------------------------------------------------------------------
void RoController::readSensors(uint32_t now) {
    r_.pressIn_raw   = eq_.readPin(ch::PRESSURE_IN);
    r_.pressOut_raw  = eq_.readPin(ch::PRESSURE_OUT);
    r_.cleanTank_raw = eq_.readPin(ch::CLEAN_TANK);
    r_.supplyTank_raw= eq_.readPin(ch::SUPPLY_TANK);
    r_.tdsIn_mv      = eq_.readPin(ch::TDS_IN);
    r_.tdsOut_mv     = eq_.readPin(ch::TDS_OUT);
    r_.motorTemp_raw = eq_.readPin(ch::MOTOR_NTC);
    r_.runSwitch     = eq_.readPin(ch::RUN_STOP) != 0;   // contact closed = RUN

    r_.pressIn_psi   = psi_from_cin(r_.pressIn_raw);
    r_.pressOut_psi  = psi_from_cin(r_.pressOut_raw);
    r_.cleanTank_cm  = cm_from_cin(r_.cleanTank_raw);
    r_.supplyTank_cm = cm_from_cin(r_.supplyTank_raw);
    r_.tdsIn_mgL     = tdsIn_from_ain(r_.tdsIn_mv);
    r_.tdsOut_mgL    = tdsOut_from_ain(r_.tdsOut_mv);
    r_.motorTemp_c   = celsius_from_tin(r_.motorTemp_raw);

    r_.feedFlow_lpm     = feedFlow_.update(eq_.readPin(ch::FLOW_IN), now);
    r_.permeateFlow_lpm = permeateFlow_.update(eq_.readPin(ch::FLOW_OUT), now);
    r_.recovery_pct = (r_.feedFlow_lpm > 0.01f)
                          ? (r_.permeateFlow_lpm / r_.feedFlow_lpm) * 100.0f
                          : 0.0f;

    r_.sensorValid =
        !cinFaulted(r_.pressIn_raw,   p_.brokenWire_mA) &&
        !cinFaulted(r_.pressOut_raw,  p_.brokenWire_mA) &&
        !cinFaulted(r_.cleanTank_raw, p_.brokenWire_mA) &&
        !cinFaulted(r_.supplyTank_raw,p_.brokenWire_mA) &&
        tinValid(r_.motorTemp_raw);
}

// ---------------------------------------------------------------------------
// Safety faults — always active (even during the start settling window, spec
// §6.4). Returns the first detected fault, NONE otherwise.
// ---------------------------------------------------------------------------
Fault RoController::detectSafetyFault(uint32_t now) {
    // Broken-wire / over-current on any 4-20 mA loop, or NTC open/short (spec
    // §7/§10). A faulted sensor makes the other readings meaningless, so this
    // is checked first.
    if (!r_.sensorValid) { dryRun_.reset(); return Fault::SENSOR; }

    // Motor over-temperature (spec §7/§10, code 1).
    if (r_.motorTemp_c >= p_.motorTempLimit_c) { dryRun_.reset(); return Fault::MOTOR_OVERTEMP; }

    // Over-pressure on either transmitter (spec §8/§10, code 4).
    if (r_.pressIn_psi > p_.overPressure_psi || r_.pressOut_psi > p_.overPressure_psi) {
        dryRun_.reset();
        return Fault::OVERPRESSURE;
    }

    // Dry-run: pump commanded ON and feed flow ~0 past the grace time (spec
    // §7/§10, code 2). Uses last tick's pump command.
    bool dryCond = prevPump_ && (r_.feedFlow_lpm < p_.dryRunFlow_lpm);
    if (dryRun_.update(dryCond, now, p_.dryRunGrace_ms)) return Fault::DRY_RUN;

    return Fault::NONE;
}

// TDS over limit sustained for the fault delay — production fault, evaluated
// only while RUNNING (masked during settling, spec §6.4 / §7 / §10, code 3).
bool RoController::tdsFaultActive(uint32_t now) {
    bool over = r_.tdsOut_mgL > p_.tdsLimit_mgL;
    return tdsFault_.update(over, now, p_.tdsFaultDelay_ms);
}

// ---------------------------------------------------------------------------
// Start permissives (spec §6.3).
// ---------------------------------------------------------------------------
bool RoController::permissivesMet(uint32_t now) const {
    return r_.runSwitch &&
           fault_ == Fault::NONE &&
           r_.supplyTank_cm >= p_.supplyMin_cm &&
           r_.cleanTank_cm  <  p_.cleanRestart_cm &&
           r_.motorTemp_c   <  p_.motorTempLimit_c &&
           (uint32_t)(now - pumpOffSince_) >= p_.minOffTime_ms;
}

// From a stopped/holding state with the selector in RUN, either begin the start
// sequence or settle into the correct hold state for annunciation.
void RoController::startOrHold(uint32_t now) {
    if (permissivesMet(now)) { enter(State::STARTING, now); return; }
    if (r_.cleanTank_cm >= p_.cleanRestart_cm) { enter(State::STOPPED_TANK_FULL, now); return; }
    if (r_.supplyTank_cm < p_.supplyMin_cm)    { enter(State::PAUSED_SUPPLY_LOW, now); return; }
    // Otherwise we are simply waiting on the min OFF-time; hold in a stopped
    // state. Tank-full is the conventional "ready but idle" indication.
    if (state_ != State::STOPPED_TANK_FULL && state_ != State::PAUSED_SUPPLY_LOW)
        enter(State::STOPPED_TANK_FULL, now);
}

// ---------------------------------------------------------------------------
// Operational state machine (no fault, selector = RUN).
// ---------------------------------------------------------------------------
void RoController::runStateMachine(uint32_t now) {
    const bool minOnElapsed = (uint32_t)(now - pumpOnSince_) >= p_.minOnTime_ms;

    switch (state_) {
        case State::STOP_ESTOP:           // selector just moved to RUN
            startOrHold(now);             // cold-start permissive (supply >= min)
            break;

        case State::STOPPED_TANK_FULL:
            // Restart only once the level has fallen below the restart
            // threshold (hysteresis, spec §6.7: stop 400, restart 380).
            if (r_.cleanTank_cm < p_.cleanRestart_cm) startOrHold(now);
            break;

        case State::PAUSED_SUPPLY_LOW:
            // Resume only once the level has recovered to the resume threshold
            // (hysteresis, spec §6.8: pause < 50, resume 60).
            if (r_.supplyTank_cm >= p_.supplyResume_cm) startOrHold(now);
            break;

        case State::STARTING:
            // Pump on, supply open, flush closed; production alarms masked.
            // Leave for RUNNING once the settling window elapses (spec §6.4).
            if ((uint32_t)(now - stateSince_) >= p_.settleWindow_ms)
                enter(State::RUNNING, now);
            break;

        case State::RUNNING:
            // Non-safety stops respect the minimum ON-time (spec §6.5).
            if (minOnElapsed && r_.cleanTank_cm >= p_.cleanFull_cm) {
                enter(State::STOPPED_TANK_FULL, now);            // spec §6.7
            } else if (minOnElapsed && r_.supplyTank_cm < p_.supplyMin_cm) {
                enter(State::PAUSED_SUPPLY_LOW, now);            // spec §6.8
            } else if ((uint32_t)(now - runSince_) >= p_.flushInterval_ms) {
                enter(State::FLUSHING, now);                     // spec §6.6
            }
            break;

        case State::FLUSHING:
            // Pump on, supply open, flush open (spec §6.6). Supply-low still
            // pauses (no water to flush); tank level is irrelevant (brine -> drain).
            if (r_.supplyTank_cm < p_.supplyMin_cm) {
                enter(State::PAUSED_SUPPLY_LOW, now);
            } else if ((uint32_t)(now - stateSince_) >= p_.flushDuration_ms) {
                enter(State::RUNNING, now);                      // resets run timer
            }
            break;

        case State::FAULT:
            break;  // handled before this function is reached
    }
}

// ---------------------------------------------------------------------------
// Production warnings (spec §7/§8/§9). Evaluated only while RUNNING so they are
// naturally masked during the start settling window and during a flush.
// ---------------------------------------------------------------------------
void RoController::updateWarnings(uint32_t now) {
    if (state_ != State::RUNNING) {
        warn_ = {};
        pressWarn_.reset(); flowWarn_.reset(); tdsWarn_.reset();
        return;
    }

    // Pressure out of band on either transmitter, incl. > 200 PSI early fouling.
    bool pressCond = r_.pressIn_psi  < p_.pressBandLo_psi || r_.pressIn_psi  > p_.pressBandHi_psi ||
                     r_.pressOut_psi < p_.pressBandLo_psi || r_.pressOut_psi > p_.pressBandHi_psi;

    // Feed flow out of band, or recovery out of range.
    bool flowCond = r_.feedFlow_lpm < p_.feedFlowLo_lpm || r_.feedFlow_lpm > p_.feedFlowHi_lpm ||
                    r_.recovery_pct < p_.recoveryLo_pct || r_.recovery_pct > p_.recoveryHi_pct;

    // Permeate TDS rising toward the limit, or cleaning-required (high feed
    // pressure with low permeate flow, spec §8).
    bool cleaning = r_.pressIn_psi > p_.fouling_psi && r_.permeateFlow_lpm < p_.lowPermeate_lpm;
    bool tdsCond = r_.tdsOut_mgL > p_.tdsWarn_mgL || cleaning;

    warn_.pressure   = pressWarn_.update(pressCond, now, p_.warnDebounce_ms);
    warn_.flowOrRec  = flowWarn_.update(flowCond,  now, p_.warnDebounce_ms);
    warn_.tdsOrClean = tdsWarn_.update(tdsCond,    now, p_.warnDebounce_ms);
}

// ---------------------------------------------------------------------------
// Drive the actuators from the current state and refresh the RGB LED.
// ---------------------------------------------------------------------------
void RoController::writeOutputs(uint32_t now) {
    switch (state_) {
        case State::STARTING:
        case State::RUNNING:
            pump_ = true;  supply_ = true;  flush_ = false; break;
        case State::FLUSHING:
            pump_ = true;  supply_ = true;  flush_ = true;  break;
        default: // STOP_ESTOP, FAULT, STOPPED_TANK_FULL, PAUSED_SUPPLY_LOW
            pump_ = false; supply_ = false; flush_ = false; break;
    }

    // Pump edge bookkeeping for the min on/off and dry-run timers.
    if (pump_ && !prevPump_) pumpOnSince_ = now;
    if (!pump_ && prevPump_) pumpOffSince_ = now;
    prevPump_ = pump_;

    eq_.pinValue(ch::PUMP,         pump_   ? PIN_ON : PIN_OFF);
    eq_.pinValue(ch::SUPPLY_VALVE, supply_ ? PIN_ON : PIN_OFF);
    eq_.pinValue(ch::FLUSH_VALVE,  flush_  ? PIN_ON : PIN_OFF);

    rgb_ = resolveRgb(state_, fault_, warn_, now, timing_);
    eq_.pinValue(ch::LED_R, rgb_.r ? PIN_ON : PIN_OFF);
    eq_.pinValue(ch::LED_G, rgb_.g ? PIN_ON : PIN_OFF);
    eq_.pinValue(ch::LED_B, rgb_.b ? PIN_ON : PIN_OFF);
}

// ---------------------------------------------------------------------------
// One control cycle.
// ---------------------------------------------------------------------------
void RoController::tick(uint32_t now) {
    if (!initialized_) reset(now);

    readSensors(now);

    // Emergency stop / reset (spec §6.1). STOP de-energises everything and
    // clears any latched fault, bypassing the minimum ON-time.
    if (!r_.runSwitch) {
        fault_ = Fault::NONE;
        warn_ = {};
        dryRun_.reset(); tdsFault_.reset();
        if (state_ != State::STOP_ESTOP) enter(State::STOP_ESTOP, now);
        writeOutputs(now);
        return;
    }

    // A latched fault stays latched until the selector is moved to STOP (handled
    // above). Keep outputs safe; keep animating the red blink-code.
    if (state_ == State::FAULT) {
        writeOutputs(now);
        return;
    }

    // Safety faults act immediately, bypassing the minimum ON-time (spec §6.5/§10).
    Fault sf = detectSafetyFault(now);
    if (sf != Fault::NONE) {
        latch(sf, now);
        writeOutputs(now);
        return;
    }

    // Permeate out-of-spec is a production fault: only while RUNNING (spec §6.4).
    if (state_ == State::RUNNING && tdsFaultActive(now)) {
        latch(Fault::PERMEATE_TDS, now);
        writeOutputs(now);
        return;
    }

    runStateMachine(now);
    updateWarnings(now);
    writeOutputs(now);
}

} // namespace ro
