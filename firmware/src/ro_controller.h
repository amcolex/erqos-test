// ro_controller.h — the RO skid control state machine (spec §5-§11).
//
// PLATFORM-INDEPENDENT. Depends only on IEqsp32 (the readPin/pinValue port) and
// an injected millisecond clock (passed to tick()), so it runs identically on
// the EQSP32 and on a host test rig. No <Arduino.h>, no millis(), no delay().
#pragma once

#include <cstdint>
#include "eqsp32_port.h"
#include "ro_types.h"
#include "sensors.h"
#include "rgb_indicator.h"

namespace ro {

class RoController {
public:
    explicit RoController(IEqsp32& eq, const Params& p = {}, const RgbTiming& timing = {});

    // Initialise internal state for power-on (spec §6.2). Call once before tick().
    void reset(uint32_t now_ms);

    // Run one control cycle. Reads sensors, evaluates logic, writes outputs.
    void tick(uint32_t now_ms);

    // --- Observability (diagnostics + tests) -------------------------------
    State    state()    const { return state_; }
    Fault    fault()    const { return fault_; }
    Warnings warnings() const { return warn_; }
    Rgb      rgb()      const { return rgb_; }
    const Readings& readings() const { return r_; }
    bool pumpCmd()        const { return pump_; }
    bool supplyValveCmd() const { return supply_; }
    bool flushValveCmd()  const { return flush_; }
    const Params& params() const { return p_; }

private:
    // Pipeline steps (in tick() order).
    void readSensors(uint32_t now);
    Fault detectSafetyFault(uint32_t now);
    void runStateMachine(uint32_t now);
    void updateWarnings(uint32_t now);
    void writeOutputs(uint32_t now);

    // Helpers.
    void enter(State s, uint32_t now);
    void latch(Fault f, uint32_t now);
    bool permissivesMet(uint32_t now) const;
    void startOrHold(uint32_t now);     // from a stopped state, decide where to go
    bool tdsFaultActive(uint32_t now);  // production fault, RUNNING only

    IEqsp32&  eq_;
    Params    p_;
    RgbTiming timing_;

    State    state_ = State::STOP_ESTOP;
    Fault    fault_ = Fault::NONE;
    Warnings warn_{};
    Rgb      rgb_{};
    Readings r_{};

    // Output command cache.
    bool pump_ = false, supply_ = false, flush_ = false, prevPump_ = false;

    // Timestamps (wrap-safe deltas via unsigned subtraction).
    uint32_t stateSince_  = 0;  // entered current state
    uint32_t pumpOnSince_ = 0;  // pump last transitioned off->on
    uint32_t pumpOffSince_= 0;  // pump last transitioned on->off
    uint32_t runSince_    = 0;  // entered RUNNING (resets the flush interval)

    // Sustained-condition trackers.
    Debouncer dryRun_;     // feed flow ~0 with pump ON -> FAULT (grace 10 s)
    Debouncer tdsFault_;   // permeate TDS over limit -> FAULT (sustained 10 min)
    Debouncer pressWarn_;  // pressure out of band / fouling -> warning
    Debouncer flowWarn_;   // flow or recovery out of band -> warning
    Debouncer tdsWarn_;    // permeate TDS rising / cleaning required -> warning

    FlowRate feedFlow_;
    FlowRate permeateFlow_;

    bool initialized_ = false;
};

} // namespace ro
