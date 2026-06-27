// sketch.h — test-side handle on the sketch under test (firmware.ino).
//
// The sketch is compiled once (sketch_tu.cpp) against the fakes. Tests reach it
// through these externs and reset it between cases with sketchReset().
#pragma once

#include "Arduino.h"   // millis(), g_fakeMillis
#include "EQSP32.h"    // the fake EQSP32 type

void setup();              // firmware.ino
void loop();               // firmware.ino
extern EQSP32 eqsp32;      // the global device object in firmware.ino

// Fresh start for a test: virtual clock to t0, device to defaults, run setup().
inline void sketchReset(uint32_t t0 = 0) {
    g_fakeMillis = t0;
    eqsp32 = EQSP32{};
    setup();
}
