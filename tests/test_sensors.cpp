// test_sensors.cpp — sensor scaling, fault sentinels, and the flow integrator.
#include <doctest/doctest.h>
#include "sensors.h"

using namespace ro;

TEST_CASE("pressure scaling: 4 mA = 0 PSI, 20 mA = 300 PSI (spec §4)") {
    CHECK(psi_from_cin(400)  == doctest::Approx(0.0f));     // 4 mA
    CHECK(psi_from_cin(2000) == doctest::Approx(300.0f));   // 20 mA
    CHECK(psi_from_cin(1200) == doctest::Approx(150.0f));   // 12 mA = mid-scale
    // Operating band top (200 PSI) and over-pressure trip (250 PSI).
    CHECK(psi_from_cin(1467) == doctest::Approx(200.0f).epsilon(0.01));
    CHECK(psi_from_cin(1733) == doctest::Approx(250.0f).epsilon(0.01));
}

TEST_CASE("depth scaling: 4 mA = 0 cm, 20 mA = 500 cm (spec §4)") {
    CHECK(cm_from_cin(400)  == doctest::Approx(0.0f));
    CHECK(cm_from_cin(2000) == doctest::Approx(500.0f));
    CHECK(cm_from_cin(1200) == doctest::Approx(250.0f));
}

TEST_CASE("TDS scaling (spec §4)") {
    CHECK(tdsIn_from_ain(0)    == doctest::Approx(0.0f));
    CHECK(tdsIn_from_ain(5000) == doctest::Approx(10000.0f));  // feed 0-10000
    CHECK(tdsIn_from_ain(2500) == doctest::Approx(5000.0f));
    CHECK(tdsOut_from_ain(0)    == doctest::Approx(0.0f));
    CHECK(tdsOut_from_ain(5000) == doctest::Approx(1000.0f));  // permeate 0-1000
    CHECK(tdsOut_from_ain(1000) == doctest::Approx(200.0f));   // the TDS limit
}

TEST_CASE("NTC temperature scaling + validity (spec §4/§7)") {
    CHECK(celsius_from_tin(250) == doctest::Approx(25.0f));
    CHECK(celsius_from_tin(800) == doctest::Approx(80.0f));   // the limit
    CHECK(tinValid(250));
    CHECK_FALSE(tinValid(TIN_OPEN));
    CHECK_FALSE(tinValid(TIN_SHORT));
}

TEST_CASE("4-20 mA broken-wire / over-current detection (spec §7/§10)") {
    CHECK_FALSE(cinFaulted(400));            // 4.00 mA ok
    CHECK_FALSE(cinFaulted(2000));           // 20 mA ok
    CHECK_FALSE(cinFaulted(350));            // exactly 3.5 mA = ok edge
    CHECK(cinFaulted(349));                  // 3.49 mA = broken wire
    CHECK(cinFaulted(0));                    // dead loop
    CHECK(cinFaulted(CIN_OVER_CURRENT));     // -1 over-current sentinel
}

TEST_CASE("FlowRate integrates pulses to L/min (spec §4: 100 pulses = 1 L)") {
    FlowRate fr(100.0f, 1000);
    fr.update(0, 0);                          // prime at t=0
    // 100 pulses delivered over exactly 60 s -> 1 L/min.
    CHECK(fr.update(100, 60000) == doctest::Approx(1.0f));

    SUBCASE("value is held between window completions") {
        // 500 ms later, window not complete -> previous value retained.
        CHECK(fr.update(0, 60500) == doctest::Approx(1.0f));
    }

    SUBCASE("steady 10 L/min") {
        FlowRate f(100.0f, 1000);
        f.update(0, 0);
        // 10 L/min = 1000 pulses/min ~= 16.67 pulses per 1 s window.
        float last = 0;
        for (uint32_t t = 1000; t <= 10000; t += 1000) last = f.update(17, t);
        CHECK(last == doctest::Approx(10.2f).epsilon(0.05));
    }
}
