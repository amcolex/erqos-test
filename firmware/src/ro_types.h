// ro_types.h — core enums, structs and tunable parameters for the RO skid.
//
// This header is PLATFORM-INDEPENDENT: it must not include <Arduino.h> or
// <EQSP32.h>. The whole control core is built around that rule so it can be
// compiled and unit-tested natively on a host machine. See io_interface.h.
#pragma once

#include <cstdint>

namespace ro {

// ---------------------------------------------------------------------------
// Operating states (spec §5). "WARNING" from the spec is not a separate state;
// it is RUNNING with one or more Warnings set, surfaced through the RGB LED
// (spec §9). This keeps the state machine and the annunciation orthogonal.
// ---------------------------------------------------------------------------
enum class State : uint8_t {
    STOP_ESTOP = 0,     // Run/Stop selector in STOP (or no power). All outputs off.
    STOPPED_TANK_FULL,  // Clean-water tank full; waiting to drain.
    PAUSED_SUPPLY_LOW,  // Supply tank low; waiting for refill.
    STARTING,           // Permissives met; pump on, pressure/flow settling.
    RUNNING,            // Producing within spec.
    FLUSHING,           // Periodic forward-flush (brine bypass open).
    FAULT,              // Latched error; safe state until reset via STOP.
};

// ---------------------------------------------------------------------------
// Latched faults (spec §10). The numeric value IS the red blink-code.
// ---------------------------------------------------------------------------
enum class Fault : uint8_t {
    NONE           = 0,
    MOTOR_OVERTEMP = 1,  // NTC >= limit
    DRY_RUN        = 2,  // Feed flow ~0 with pump ON past the grace time
    PERMEATE_TDS   = 3,  // TDS OUT over limit sustained
    OVERPRESSURE   = 4,  // Pressure IN/OUT over the safety trip
    SENSOR         = 5,  // 4-20 mA broken wire / over-current, or NTC open/short
};

// ---------------------------------------------------------------------------
// Marginal-but-running conditions (spec §7/§8/§9). Mapped to the three yellow
// blink rates by RgbIndicator (fast > medium > slow when several are active).
// ---------------------------------------------------------------------------
struct Warnings {
    bool pressure   = false;  // pressure out of band / early fouling   -> slow blink
    bool flowOrRec  = false;  // flow or recovery out of band           -> medium blink
    bool tdsOrClean = false;  // permeate TDS rising / cleaning required -> fast blink
    bool any() const { return pressure || flowOrRec || tdsOrClean; }
    bool operator==(const Warnings& o) const {
        return pressure == o.pressure && flowOrRec == o.flowOrRec &&
               tdsOrClean == o.tdsOrClean;
    }
};

// Three independent on/off LED channels (spec §3 DIO11-13). Explicit constexpr
// constructor so brace-initialisation works under C++11 (ESP32 core default),
// where a struct with default member initializers is not an aggregate.
struct Rgb {
    bool r, g, b;
    constexpr Rgb(bool r_ = false, bool g_ = false, bool b_ = false)
        : r(r_), g(g_), b(b_) {}
    bool operator==(const Rgb& o) const { return r == o.r && g == o.g && b == o.b; }
    bool operator!=(const Rgb& o) const { return !(*this == o); }
};

// ---------------------------------------------------------------------------
// Tunable setpoints — every value here is a spec §11 default. The spec §13
// "open points to confirm" all reduce to editing one of these fields.
// ---------------------------------------------------------------------------
struct Params {
    // Tank levels (cm), with hysteresis (spec §6.7/§6.8/§11).
    float supplyMin_cm     = 50.0f;   // below -> pause
    float supplyResume_cm  = 60.0f;   // resume at/above
    float cleanFull_cm     = 400.0f;  // stop production at/above
    float cleanRestart_cm  = 380.0f;  // restart below

    // Pressure (PSI) (spec §4/§7/§8/§11).
    float pressBandLo_psi  = 150.0f;
    float pressBandHi_psi  = 200.0f;
    float fouling_psi      = 200.0f;  // feed pressure above this = fouling indication
    float overPressure_psi = 250.0f;  // safety trip

    // Permeate TDS (mg/L) (spec §7/§11).
    float    tdsLimit_mgL    = 200.0f;
    float    tdsWarn_mgL     = 180.0f;            // "rising toward 200" warning
    uint32_t tdsFaultDelay_ms = 10u * 60u * 1000u; // sustained over limit -> FAULT

    // Flow (L/min) (spec §7/§11).
    float    feedFlowLo_lpm   = 8.0f;
    float    feedFlowHi_lpm   = 12.0f;
    float    dryRunFlow_lpm   = 0.5f;          // "~0" threshold (spec §13 open point)
    uint32_t dryRunGrace_ms   = 10u * 1000u;   // pump ON this long with ~0 flow -> FAULT
    float    lowPermeate_lpm  = 1.0f;          // "permeate flow is low" for cleaning-required

    // Recovery (%) = permeate/feed (spec §7/§11).
    float recoveryLo_pct = 10.0f;
    float recoveryHi_pct = 30.0f;

    // Motor temperature (deg C) (spec §7/§11).
    float motorTempLimit_c = 80.0f;

    // Sequence timers (spec §6/§11).
    uint32_t flushInterval_ms = 60u * 60u * 1000u; // continuous run before a flush
    uint32_t flushDuration_ms = 5u * 60u * 1000u;
    uint32_t minOnTime_ms     = 10u * 1000u;       // before a non-safety stop
    uint32_t minOffTime_ms    = 10u * 1000u;       // before a restart
    uint32_t settleWindow_ms  = 25u * 1000u;       // production alarms masked

    // Sensor fault (spec §7/§10).
    float brokenWire_mA = 3.5f;  // 4-20 mA below this = broken wire

    // Warning debounce (spec §7 "debounce timers to avoid nuisance reactions").
    uint32_t warnDebounce_ms = 3000;

    // Behaviour flag (spec §13 open point 6): default warn-only on cleaning-required.
    bool cleaningRequiredStops = false;
};

// ---------------------------------------------------------------------------
// Debouncer — asserts only after a condition has held continuously for `ms`.
// Used for warning suppression and the timed faults (dry-run 10 s, TDS 10 min).
// Wrap-safe. Pure; no hardware dependency.
// ---------------------------------------------------------------------------
struct Debouncer {
    uint32_t since = 0;
    bool counting = false;
    bool latched = false;

    // Returns true once `cond` has been true continuously for >= `ms`.
    bool update(bool cond, uint32_t now, uint32_t ms) {
        if (!cond) { counting = false; latched = false; return false; }
        if (!counting) { counting = true; since = now; }
        if ((uint32_t)(now - since) >= ms) latched = true;
        return latched;
    }
    uint32_t heldFor(uint32_t now) const { return counting ? (uint32_t)(now - since) : 0; }
    void reset() { counting = false; latched = false; since = 0; }
};

// Snapshot of the most recent sensor reads, in both raw and engineering units.
// Exposed for diagnostics and asserted directly in tests.
struct Readings {
    // Raw, exactly as EQSP32 readPin() would return.
    int pressIn_raw  = 0;  // mA x 100
    int pressOut_raw = 0;  // mA x 100
    int cleanTank_raw = 0; // mA x 100
    int supplyTank_raw = 0;// mA x 100
    int tdsIn_mv  = 0;     // mV
    int tdsOut_mv = 0;     // mV
    int motorTemp_raw = 0; // deg C x 10 (or TIN sentinel)
    // Scaled engineering units.
    float pressIn_psi   = 0;
    float pressOut_psi  = 0;
    float cleanTank_cm  = 0;
    float supplyTank_cm = 0;
    float tdsIn_mgL     = 0;
    float tdsOut_mgL    = 0;
    float motorTemp_c   = 0;
    float feedFlow_lpm  = 0;
    float permeateFlow_lpm = 0;
    float recovery_pct  = 0;
    bool  runSwitch = false;       // true = RUN
    bool  sensorValid = true;      // false if any analog sensor faulted
};

} // namespace ro
