// sensors.cpp — FlowRate implementation (everything else in sensors.h is inline).
#include "sensors.h"

namespace ro {

void FlowRate::reset() {
    winStart_ = 0;
    pulseAcc_ = 0;
    lpm_ = 0.0f;
    primed_ = false;
}

float FlowRate::update(int pulses, uint32_t now_ms) {
    if (!primed_) {
        winStart_ = now_ms;
        pulseAcc_ = 0;
        primed_ = true;
    }
    pulseAcc_ += pulses;

    uint32_t dt = now_ms - winStart_;   // wrap-safe unsigned delta
    if (dt >= windowMs_ && dt > 0) {
        float liters = pulseAcc_ / pulsesPerLiter_;
        float minutes = dt / 60000.0f;
        lpm_ = liters / minutes;
        winStart_ = now_ms;
        pulseAcc_ = 0;
    }
    return lpm_;
}

} // namespace ro
