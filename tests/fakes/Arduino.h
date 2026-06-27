// Arduino.h — minimal host fake of the Arduino runtime, for compiling the sketch
// natively. Provides a test-controlled millisecond clock plus no-op Serial/delay.
#pragma once

#include <cstdint>
#include <cstddef>

// Virtual clock, driven by the test harness (see sketch.h / harness.h).
extern uint32_t g_fakeMillis;
inline uint32_t millis() { return g_fakeMillis; }
inline uint32_t micros() { return g_fakeMillis * 1000u; }
inline void delay(uint32_t) {}              // no-op: the harness owns time
inline void delayMicroseconds(uint32_t) {}

#define HIGH 1
#define LOW  0
#define INPUT  0
#define OUTPUT 1

// Just enough Serial for the sketch's status report (begin + printf).
struct FakeSerial {
    void begin(unsigned long) {}
    void println(const char* = "") {}
    void print(const char* = "") {}
    template <class... A> int printf(const char*, A...) { return 0; }
};
extern FakeSerial Serial;
