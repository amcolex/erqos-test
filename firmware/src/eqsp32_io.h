// eqsp32_io.h — production ISkidIO adapter backed by the EQSP32.
//
// This is the boundary between portable control logic and the vendor library.
// EQSP32.h is included only in the .cpp, so this header stays host-clean (the
// EQSP32 type is forward-declared). This file (and firmware.ino) are the ONLY
// places that depend on EQSP32.
#pragma once

#include "io_interface.h"

class EQSP32;  // vendor type, global namespace

namespace ro {

class Eqsp32SkidIO : public ISkidIO {
public:
    explicit Eqsp32SkidIO(EQSP32& eq) : eq_(eq) {}

    // Configure pin modes per the spec §3 channel map. Call AFTER eqsp32.begin().
    void begin();

    // --- ISkidIO inputs ----------------------------------------------------
    int  readPressureIn()    override;
    int  readPressureOut()   override;
    int  readCleanTank()     override;
    int  readSupplyTank()    override;
    int  readTdsIn()         override;
    int  readTdsOut()        override;
    int  readMotorTemp()     override;
    bool readRunSwitch()     override;
    int  readFeedPulses()    override;
    int  readPermeatePulses()override;

    // --- ISkidIO outputs ---------------------------------------------------
    void setPump(bool on)          override;
    void setSupplyValve(bool open) override;
    void setFlushValve(bool open)  override;
    void setRgb(bool r, bool g, bool b) override;

private:
    EQSP32& eq_;
};

} // namespace ro
