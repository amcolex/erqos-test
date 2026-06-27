# erqos-firmware — RO Skid Controller

Firmware for a single-pass **reverse-osmosis water-treatment skid** driven by one
**Erqos EQSP32** (ESP32-S3 industrial PLC). It implements
[`RO-Skid-Control-Specification.docx`](RO-Skid-Control-Specification.docx): an
8-state machine with safety interlocks, latched faults, timed sequences
(settling, periodic flush, TDS trip, min on/off), level/pressure hysteresis,
membrane-fouling detection and a priority-based RGB status indicator.

The dev environment is managed by [pixi](https://pixi.sh). `pixi run setup` pulls
a pinned, self-contained Arduino toolchain (nothing touches your global install);
`pixi run test` builds and runs the control logic natively on your machine.

## What's here

```
.
├── pixi.toml                  # environment + tasks (start here)
├── firmware/
│   └── firmware.ino           # THE WHOLE PROGRAM: config, state machine, setup(), loop()
├── tests/                     # native test suite (doctest) — no hardware
│   ├── fakes/
│   │   ├── EQSP32.h           # fake EQSP32: scriptable readPin / recorded pinValue
│   │   └── Arduino.h          # fake millis()/delay()/Serial (virtual clock)
│   ├── sketch_tu.cpp          # compiles firmware.ino against the fakes
│   ├── support/plant_model.*  # virtual RO skid physics for scenario tests
│   ├── support/harness.h      # drive setup()/loop(); observe outputs + LED
│   ├── test_behaviors.cpp     # scripted-input black-box checks + threshold tests
│   └── test_scenarios.cpp     # closed-loop end-to-end scenarios
├── EQSP32/                    # vendor library (Erqos/EQSP32), used in place
├── scripts/                   # task implementations invoked by pixi
└── .arduino/  build/          # toolchain, cores & build output (git-ignored)
```

## Architecture — a normal sketch that's still fully testable

`firmware.ino` is an ordinary Arduino sketch: setpoints, a state machine, helper
functions, `setup()` and `loop()`, talking to `eqsp32.readPin()/pinValue()` and
`millis()`. Nothing is split into classes or extra files.

It's testable because the **seam is the build, not the code**. The host test
build puts fake [`EQSP32.h`](tests/fakes/EQSP32.h) + [`Arduino.h`](tests/fakes/Arduino.h)
on the include path and compiles the *exact same sketch* against them
([sketch_tu.cpp](tests/sketch_tu.cpp)):

- the **fake EQSP32** lets tests script sensor readings and record the
  pump/valve/LED commands;
- the **fake `millis()`** is a virtual clock the harness advances, so a 60-minute
  flush or a 10-minute TDS trip runs in microseconds, deterministically.

Tests are **black-box**: they drive the real `setup()`/`loop()` and assert on the
observable outputs — the pump/valve commands and the RGB LED, which by spec §9
encodes the operating state (e.g. solid green = running in spec, blue blink =
supply-low pause, red N-blinks = fault code N). That verifies operator-visible
behaviour, not internal variables.

## Testing

Both suites run with `pixi run test` (no hardware, runs in well under a second):

- **`test_behaviors.cpp`** — scripted inputs (a frozen plant): power-on,
  permissives, e-stop/reset, each fault + red blink-code, and threshold checks
  that pin the spec §11 setpoints (49 vs 50 cm, 79 vs 80 °C, 250 vs 251 PSI, …).
- **`test_scenarios.cpp`** — a `PlantModel` turns the pump/valve commands into
  pressure/flow/level/temperature dynamics fed back as sensor readings, so
  scenarios read like real operation: power-on → start → run → tank-full stop →
  drain → restart; periodic flush; supply-low pause/resume; dry-run, over-
  pressure, over-temp, TDS and broken-wire faults; warn-only fouling; reset.

Current: **26 test cases / 98 assertions, ~99% line + 100% function coverage**
of `firmware.ino` (`pixi run coverage`).

## Quick start

```bash
pixi run setup      # one-time: fetch arduino-cli, ESP32 core 2.0.17, RadioLib (~250 MB)
pixi run test       # build + run the control-logic test suite (no hardware)
pixi run build      # compile the firmware for the EQSP32
# plug the EQSP32 in over USB, then:
pixi run flash      # build + upload
pixi run monitor    # serial console @ 115200 baud (one-line status per second)
```

## Tasks

| Command            | Does |
|--------------------|------|
| `pixi run setup`   | Install the toolchain into `.arduino/` (idempotent). |
| `pixi run test`    | Build & run the native unit + scenario suite. |
| `pixi run coverage`| Run tests and print a coverage report for the control core. |
| `pixi run build`   | Compile `firmware/` for the EQSP32. |
| `pixi run upload`  | Flash the last build (auto-detects the port, or set `PORT`). |
| `pixi run flash`   | `build` then `upload`. |
| `pixi run monitor` | Open the serial monitor at 115200 baud. |
| `pixi run boards`  | List connected boards / serial ports. |
| `pixi run clean`   | Remove build artifacts. |

## Tuning setpoints

Every spec §11 setpoint is a named `constexpr` near the top of
[firmware.ino](firmware/firmware.ino) (the "Configuration" section). The spec §13
"open points to confirm" (flow band, dry-run threshold, flash-rate mapping,
debounce, fouling-stop behaviour) are all one-line edits there; the threshold
tests in `test_behaviors.cpp` lock the values down so changes are deliberate.
Board/port/version knobs live in the `[activation.env]` block of `pixi.toml`.

## Toolchain

This is an **Arduino library project**, so we build it the way Erqos ships it —
`arduino-cli` + the official ESP32 Arduino core, driven by pixi.

| Component        | Version  | Why pinned |
|------------------|----------|------------|
| arduino-cli      | 1.1.1    | reproducible CLI |
| esp32 Arduino core | **2.0.17** | EQSP32's `libEQSP32.a` is precompiled against the **v2** core. Do **not** move to 3.x — Erqos states the library "supports up to esp32 v2 core" and the ABI would break. |
| RadioLib         | 7.7.1    | only third-party header `EQSP32.h` includes (LoRa) |
| Board (FQBN)     | `esp32:esp32:esp32s3` | ESP32S3 Dev Module |
| Flash / Partition | `8M` / `8M with spiffs` (`default_8MB`) | per Erqos quickstart guide |
| host tests       | conda-forge `cxx-compiler` + `cmake` + `ninja` + `doctest` | native build, no hardware |

### Choosing the serial port

Auto-detection picks the only connected board. With several boards, set it:

```bash
PORT=/dev/cu.usbserial-XXXX pixi run flash    # find the name with: pixi run boards
```

## References

- Control specification: [`RO-Skid-Control-Specification.docx`](RO-Skid-Control-Specification.docx)
- EQSP32 library & examples: <https://github.com/Erqos/EQSP32>
- EQSP32 quickstart guide: <https://erqos.com/resources/quickstart-guide/>
