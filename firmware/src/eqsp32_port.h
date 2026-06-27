// eqsp32_port.h — the hardware seam.
//
// The control core talks to the EQSP32 through this 2-verb port, which mirrors
// the EQSP32 API itself (readPin / pinValue). Production wraps the real EQSP32
// (see firmware.ino); tests use FakeEqsp32. Keeping the seam at the EQSP32 verbs
// means values flow in EQSP32 readPin() native units (mA x100, mV, degC x10,
// pulses) end-to-end — no translation layer to get wrong.
//
// PLATFORM-INDEPENDENT: nothing in firmware/src/ includes <Arduino.h>/<EQSP32.h>.
#pragma once

namespace ro {

// Physical EQSP32 channel assignment (spec §3). Single source of truth.
namespace ch {
constexpr int PRESSURE_IN  = 1;   // CIN  4-20 mA  membrane feed pressure
constexpr int PRESSURE_OUT = 2;   // CIN  4-20 mA  concentrate (brine) pressure
constexpr int CLEAN_TANK   = 3;   // CIN  4-20 mA  product tank level
constexpr int SUPPLY_TANK  = 4;   // CIN  4-20 mA  feed tank level
constexpr int TDS_IN       = 5;   // AIN  0-5 V    feed TDS
constexpr int TDS_OUT      = 6;   // AIN  0-5 V    permeate TDS
constexpr int RUN_STOP     = 7;   // DIN          run enable / e-stop / reset
constexpr int MOTOR_NTC    = 8;   // TIN          pump motor temperature
constexpr int FLOW_IN      = 9;   // PCC          feed flow
constexpr int FLOW_OUT     = 10;  // PCC          permeate flow
constexpr int LED_R        = 11;  // POUT         status RGB - red
constexpr int LED_G        = 12;  // POUT         status RGB - green
constexpr int LED_B        = 13;  // POUT         status RGB - blue
constexpr int FLUSH_VALVE  = 14;  // POUT 24 V    brine-bypass / forward-flush
constexpr int PUMP         = 15;  // POUT 24 V    RO pump contactor
constexpr int SUPPLY_VALVE = 16;  // POUT 24 V    feed inlet valve
} // namespace ch

// POUT command for a channel fully on / off (0-1000 = 0-100% duty).
constexpr int PIN_ON  = 1000;
constexpr int PIN_OFF = 0;

// The two EQSP32 verbs the controller needs. Both the real EQSP32 (via a 4-line
// adapter in firmware.ino) and FakeEqsp32 satisfy this.
class IEqsp32 {
public:
    virtual ~IEqsp32() = default;
    virtual int  readPin(int pin) = 0;          // EQSP32 readPin()  (native units)
    virtual void pinValue(int pin, int value) = 0; // EQSP32 pinValue() (0..1000)
};

} // namespace ro
