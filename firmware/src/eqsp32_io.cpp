// eqsp32_io.cpp — maps the abstract skid I/O onto concrete EQSP32 calls.
#include "eqsp32_io.h"
#include "EQSP32.h"

namespace ro {

// Motor NTC characteristic (spec §4: standard 10K NTC, Beta 3435, 10k ref).
static constexpr int NTC_BETA = 3435;
static constexpr int NTC_REF_OHMS = 10000;

// POUT command for a fully-on / off channel (0-1000 = 0-100% duty).
static constexpr int ON = 1000;
static constexpr int OFF = 0;

void Eqsp32SkidIO::begin() {
    // 4-20 mA current-loop inputs (spec §3 ADIO1-4).
    eq_.pinMode(ch::PRESSURE_IN,  CIN);
    eq_.pinMode(ch::PRESSURE_OUT, CIN);
    eq_.pinMode(ch::CLEAN_TANK,   CIN);
    eq_.pinMode(ch::SUPPLY_TANK,  CIN);

    // 0-5 V analog inputs (spec §3 ADIO5-6).
    eq_.pinMode(ch::TDS_IN,  AIN);
    eq_.pinMode(ch::TDS_OUT, AIN);

    // Run/Stop selector (spec §3 ADIO7).
    eq_.pinMode(ch::RUN_STOP, DIN);

    // Motor temperature NTC (spec §3 ADIO8).
    eq_.pinMode(ch::MOTOR_NTC, TIN);
    eq_.configTIN(ch::MOTOR_NTC, NTC_BETA, NTC_REF_OHMS);

    // Pulse flow meters (spec §3 DIO9-10).
    eq_.pinMode(ch::FLOW_IN,  PCC);
    eq_.configPCC(ch::FLOW_IN, ON_RISING);
    eq_.pinMode(ch::FLOW_OUT, PCC);
    eq_.configPCC(ch::FLOW_OUT, ON_RISING);

    // Power outputs: RGB LED + valves + pump (spec §3 DIO11-16).
    eq_.pinMode(ch::LED_R,        POUT);
    eq_.pinMode(ch::LED_G,        POUT);
    eq_.pinMode(ch::LED_B,        POUT);
    eq_.pinMode(ch::FLUSH_VALVE,  POUT);
    eq_.pinMode(ch::PUMP,         POUT);
    eq_.pinMode(ch::SUPPLY_VALVE, POUT);

    // Safe initial state.
    setPump(false);
    setSupplyValve(false);
    setFlushValve(false);
    setRgb(false, false, false);
}

int  Eqsp32SkidIO::readPressureIn()    { return eq_.readPin(ch::PRESSURE_IN); }
int  Eqsp32SkidIO::readPressureOut()   { return eq_.readPin(ch::PRESSURE_OUT); }
int  Eqsp32SkidIO::readCleanTank()     { return eq_.readPin(ch::CLEAN_TANK); }
int  Eqsp32SkidIO::readSupplyTank()    { return eq_.readPin(ch::SUPPLY_TANK); }
int  Eqsp32SkidIO::readTdsIn()         { return eq_.readPin(ch::TDS_IN); }
int  Eqsp32SkidIO::readTdsOut()        { return eq_.readPin(ch::TDS_OUT); }
int  Eqsp32SkidIO::readMotorTemp()     { return eq_.readPin(ch::MOTOR_NTC); }
// Run/Stop: contact closed (HIGH) = RUN (spec §13: confirm against wiring).
bool Eqsp32SkidIO::readRunSwitch()     { return eq_.readPin(ch::RUN_STOP) != 0; }
int  Eqsp32SkidIO::readFeedPulses()    { return eq_.readPin(ch::FLOW_IN); }
int  Eqsp32SkidIO::readPermeatePulses(){ return eq_.readPin(ch::FLOW_OUT); }

void Eqsp32SkidIO::setPump(bool on)          { eq_.pinValue(ch::PUMP,         on   ? ON : OFF); }
void Eqsp32SkidIO::setSupplyValve(bool open) { eq_.pinValue(ch::SUPPLY_VALVE, open ? ON : OFF); }
void Eqsp32SkidIO::setFlushValve(bool open)  { eq_.pinValue(ch::FLUSH_VALVE,  open ? ON : OFF); }
void Eqsp32SkidIO::setRgb(bool r, bool g, bool b) {
    eq_.pinValue(ch::LED_R, r ? ON : OFF);
    eq_.pinValue(ch::LED_G, g ? ON : OFF);
    eq_.pinValue(ch::LED_B, b ? ON : OFF);
}

} // namespace ro
