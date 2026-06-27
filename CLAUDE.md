# Project notes for Claude

Firmware for the **Erqos EQSP32**, an ESP32-S3 industrial PLC. We build the
vendor's Arduino library with `arduino-cli`, orchestrated by **pixi**.

It implements `RO-Skid-Control-Specification.docx` (reverse-osmosis skid: state
machine, interlocks, latched faults, timed sequences, RGB status).

## Build, test & run
- `pixi run setup` — one-time; installs arduino-cli + ESP32 core + RadioLib into `./.arduino/` (git-ignored, self-contained; nothing global).
- `pixi run test` — build & run the native control-logic suite (no hardware). `pixi run coverage` for a coverage report.
- `pixi run build` / `upload` / `flash` / `monitor` / `boards` / `clean`.
- Task scripts live in `scripts/`; shared config is `scripts/_env.sh`; tunables
  (versions, `FQBN`, `PORT`) are in `pixi.toml` `[activation.env]`.

## Architecture — single-file sketch, tested via fake headers
- **The whole program is `firmware/firmware.ino`** in the classic sketch shape:
  config constants, enums, scaling, the state machine, `setup()`, `loop()`. It
  uses `eqsp32.readPin/pinValue` and `millis()` directly. No classes split out.
- **The seam is the build, not the code.** The host test build (`tests/`, doctest
  via CMake) compiles the *same* `firmware.ino` (`tests/sketch_tu.cpp` includes
  it) against fake `tests/fakes/EQSP32.h` + `tests/fakes/Arduino.h` (found ahead
  of the real ones on the include path). The fake EQSP32 scripts sensor readings
  and records pump/valve/LED commands; fake `millis()` is a virtual clock.
- **Tests are black-box**: drive the real `setup()`/`loop()` and assert on outputs
  (pump/valves + RGB LED, which encodes state per spec §9). They do NOT read
  internal state — the enums/`S` struct stay private to the sketch.
- All mutable state is in one `AppState S` struct that `setup()` resets in one
  line (`S = AppState{}`), so tests stay independent (each calls `sketchReset()`).
- Spec §11 setpoints are `constexpr` near the top of the .ino; `test_behaviors.cpp`
  threshold tests lock the values. §13 open points = one-line edits there.
- `tests/fakes/EQSP32.h` keeps its OWN copy of the pin map — it MUST match the
  `PIN_*` constants in firmware.ino.
- After editing firmware.ino, run BOTH `pixi run test` and `pixi run build`.

## Non-obvious constraints — read before changing the toolchain
- **Stay on ESP32 core 2.x (pinned 2.0.17).** `EQSP32/src/esp32s3/libEQSP32.a`
  is *precompiled* (`precompiled=true`, `architectures=esp32`). Erqos: "supports
  up to esp32 v2 core." Core 3.x (ESP-IDF 5) is ABI-incompatible and will break
  linking. This is also why we use arduino-cli, not PlatformIO (whose recent
  espressif32 platforms default to core 3.x and mishandle the `src/esp32s3/*.a`
  precompiled layout).
- **arduino-cli is not on conda-forge** (Go binary) — `setup.sh` downloads a
  pinned release via curl. Curl is the one real conda dependency pixi manages.
- The EQSP32 library is used **in place** from `./EQSP32` via arduino-cli's
  `--library` flag (build.sh) — no symlink/global install.
- Board settings (from Erqos quickstart): `esp32:esp32:esp32s3`, `FlashSize=8M`,
  `PartitionScheme=default_8MB` ("8M with spiffs"). PSRAM/USB-CDC left at core
  defaults (Erqos doesn't specify them).
- Only third-party header dep of `EQSP32.h` is **RadioLib**. `library.properties`
  has no `depends=`, so deps are installed explicitly in `setup.sh`.

## EQSP32 I/O facts
- `firmware/firmware.ino` is the whole sketch (folder name must match the `.ino`
  name); it reads sensors, runs the state machine, and drives outputs in `loop()`.
- API reference: top comment block of `EQSP32/src/EQSP32.h`. Pin values are
  0–1000 (= 0–100% PWM). `readPin` units: CIN = mA×100 (broken wire `<350`,
  over-current `-1`), AIN = mV, TIN = °C×10 (open `-9999` / short `9999`),
  PCC = pulse count cleared on read. CIN/AIN/TIN need pins 1–8; PCC pins 9–16.
