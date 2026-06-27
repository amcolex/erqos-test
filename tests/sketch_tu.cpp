// sketch_tu.cpp — compiles the production sketch (firmware.ino) exactly once into
// the test binary, against the fake Arduino + EQSP32 headers. This is what lets
// the tests drive the real setup()/loop() with no hardware.
#include "Arduino.h"
#include "EQSP32.h"
#include "../firmware/firmware.ino"
