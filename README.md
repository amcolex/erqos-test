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
│   ├── firmware.ino           # the ONLY EQSP32 file: adapts it to the port, pin setup, loop
│   └── src/                   # the control logic — pure C++, no vendor headers
│       ├── ro_types.h         # State / Fault / Warnings / Params (spec §11 setpoints)
│       ├── eqsp32_port.h      # IEqsp32 — the readPin/pinValue seam + channel map (spec §3)
│       ├── sensors.{h,cpp}    # sensor scaling + flow integration (spec §4)
│       ├── rgb_indicator.{h,cpp} # RGB priority + blink/fault codes (spec §9/§10)
│       └── ro_controller.{h,cpp} # the state machine (spec §5-§10)
├── tests/                     # native unit + scenario suite (doctest); FakeEqsp32 stands in
├── EQSP32/                    # vendor library (Erqos/EQSP32), used in place
├── scripts/                   # task implementations invoked by pixi
└── .arduino/  build/          # toolchain, cores & build output (git-ignored)
```

## Architecture — why it's testable

Everything under `firmware/src/` is **platform-independent C++** that never
includes `<Arduino.h>` / `<EQSP32.h>`. It reaches hardware through two seams:

1. **I/O seam** — `IEqsp32` (in [eqsp32_port.h](firmware/src/eqsp32_port.h)): a
   2-verb port that mirrors the EQSP32 itself — `readPin(pin)` / `pinValue(pin,
   value)`, in EQSP32 native units. Production is a 4-line adapter over the real
   `EQSP32` (inlined in [firmware.ino](firmware/firmware.ino)); tests use
   `FakeEqsp32`, an in-memory stand-in.
2. **Time seam** — the controller takes the current time as a parameter
   (`tick(now_ms)`); it never calls `millis()`. So a 60-minute flush timer or a
   10-minute TDS trip is exercised in microseconds, deterministically.

`firmware.ino` is the **only** file that includes `<EQSP32.h>`; it adapts the
real device to `IEqsp32` and configures the pins. The host test build compiles
the same `firmware/src/*.cpp` against `FakeEqsp32` — so if the core ever picked
up a vendor/Arduino dependency, `pixi run test` would fail. That's the guardrail.

## Testing

Two layers, both run by `pixi run test` (no hardware, milliseconds):

- **Unit tests** — sensor scaling & fault sentinels, the flow integrator, RGB
  priority + blink/fault-code patterns, and each permissive / timer / hysteresis
  edge / fault in isolation (via a scriptable `FakeEqsp32` and a manual clock).
- **Scenario tests** — a `PlantModel` simulates the skid's physics (pump+valves →
  pressure ramps, tank fills, supply drains, flow ≈ 10 L/min; flush drops
  pressure) and feeds it back through the *same* `IEqsp32` port the real device
  uses.
  Scenarios read like real operation: power-on → start → run → tank-full stop →
  drain → restart; periodic flush; supply-low pause/resume; every fault + reset.

Current: **41 test cases / 306 assertions, ~99% line + 100% function coverage**
of the control core (`pixi run coverage`).

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

Every spec §11 setpoint is a named, defaulted field in `Params`
([ro_types.h](firmware/src/ro_types.h)). The spec §13 "open points to confirm"
(flow band, dry-run threshold, flash-rate mapping, debounce, fouling-stop
behaviour) are all one-line edits there — a unit test asserts the defaults match
the spec so changes are deliberate. Board/port/version knobs live in the
`[activation.env]` block of `pixi.toml`.

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
