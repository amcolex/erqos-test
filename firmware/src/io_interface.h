// io_interface.h — the hardware abstraction seam.
//
// The control core talks to the world ONLY through ISkidIO. Getters return
// values in EQSP32 readPin() native units (mA x100, mV, degC x10, pulses) so
// that the production adapter and the host FakeSkidIO speak the exact same
// numbers.
//
// PLATFORM-INDEPENDENT: do not include <Arduino.h> / <EQSP32.h> here.
#pragma once

namespace ro {

// Physical EQSP32 channel assignment (spec §3). Single source of truth, used by
// the EQSP32 adapter.
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

// Abstract I/O port. The controller depends on this, nothing else.
class ISkidIO {
public:
    virtual ~ISkidIO() = default;

    // --- Inputs (EQSP32 readPin units) -------------------------------------
    virtual int  readPressureIn()    = 0;  // mA x100   (CIN_OC_ERROR == -1)
    virtual int  readPressureOut()   = 0;  // mA x100
    virtual int  readCleanTank()     = 0;  // mA x100
    virtual int  readSupplyTank()    = 0;  // mA x100
    virtual int  readTdsIn()         = 0;  // mV
    virtual int  readTdsOut()        = 0;  // mV
    virtual int  readMotorTemp()     = 0;  // degC x10  (-9999 open / 9999 short)
    virtual bool readRunSwitch()     = 0;  // true = RUN, false = STOP
    virtual int  readFeedPulses()    = 0;  // pulses since last call (cleared on read)
    virtual int  readPermeatePulses()= 0;  // pulses since last call (cleared on read)

    // --- Outputs -----------------------------------------------------------
    virtual void setPump(bool on)          = 0;
    virtual void setSupplyValve(bool open) = 0;
    virtual void setFlushValve(bool open)  = 0;
    virtual void setRgb(bool r, bool g, bool b) = 0;
};

} // namespace ro
