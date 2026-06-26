/**
 * @file    firmware.ino
 * @brief   Starter firmware for the Erqos EQSP32 (ESP32-S3 industrial PLC).
 *
 * A minimal but real skeleton that exercises the core EQSP32 API:
 *   - boots the EQSP32 runtime,
 *   - configures one digital input, one relay output and one PWM power output,
 *   - mirrors the input to the relay,
 *   - reads the onboard user button and supply voltage,
 *   - beeps the buzzer on boot and prints a status line every second.
 *
 * Build/flash with pixi:
 *   pixi run build
 *   pixi run flash
 *   pixi run monitor      # 115200 baud
 *
 * Pin wiring is just an example — adjust to your hardware.
 */

#include "EQSP32.h"

EQSP32 eqsp32;

// ---- Pin assignments (EQSP32 main-unit channels 1..16) ----------------------
constexpr int PIN_INPUT  = 1;   // DIN   : a dry contact / digital sensor
constexpr int PIN_RELAY  = 6;   // RELAY : an inductive load (e.g. a contactor)
constexpr int PIN_PWM    = 5;   // POUT  : a PWM-driven load (e.g. a lamp/valve)

// ---- Relay tuning -----------------------------------------------------------
constexpr int RELAY_HOLD_POWER   = 350;   // 35.0% holding power after pull-in
constexpr int RELAY_DERATE_DELAY = 1500;  // ms at full power before derating

// ---- Non-blocking status print ----------------------------------------------
constexpr unsigned long STATUS_PERIOD_MS = 1000;
unsigned long lastStatus = 0;

void setup() {
    // Start the EQSP32 runtime first (verbose = true prints boot diagnostics).
    eqsp32.begin(true);

    Serial.begin(115200);
    Serial.println("\nEQSP32 starter firmware booting...");

    // Inputs / outputs.
    eqsp32.pinMode(PIN_INPUT, DIN);
    eqsp32.pinMode(PIN_RELAY, RELAY);
    eqsp32.configRELAY(PIN_RELAY, RELAY_HOLD_POWER, RELAY_DERATE_DELAY);
    eqsp32.pinMode(PIN_PWM, POUT);

    // Safe initial state.
    eqsp32.pinValue(PIN_RELAY, 0);   // relay off
    eqsp32.pinValue(PIN_PWM, 0);     // 0% PWM (range is 0..1000 => 0..100%)

    // Short startup beep so we know the firmware is alive.
    eqsp32.buzzerOn(2000, 120);      // 2 kHz for 120 ms
}

void loop() {
    // Mirror the digital input onto the relay.
    bool inputActive = (eqsp32.readPin(PIN_INPUT) == HIGH);
    eqsp32.pinValue(PIN_RELAY, inputActive ? 1000 : 0);

    // Drive the PWM output to half power while the input is active.
    eqsp32.pinValue(PIN_PWM, inputActive ? 500 : 0);

    // Periodic status over the USB serial link.
    unsigned long now = millis();
    if (now - lastStatus >= STATUS_PERIOD_MS) {
        lastStatus = now;

        int  supply_mV = eqsp32.readInputVoltage();   // supply voltage in mV
        bool userBtn   = eqsp32.readUserButton();      // onboard BOOT button

        Serial.printf("[%lus] input=%-3s relay=%-3s Vin=%d.%03dV button=%s\n",
                      now / 1000,
                      inputActive ? "ON" : "OFF",
                      inputActive ? "ON" : "OFF",
                      supply_mV / 1000, supply_mV % 1000,
                      userBtn ? "PRESSED" : "released");
    }

    delay(20);   // cooperative pause; EQSP32 runs its services on the other core
}
