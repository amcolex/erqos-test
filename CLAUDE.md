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

## Control-logic architecture (read before editing firmware/src)
- **The seam is the point.** Control logic in `firmware/src/{ro_controller,
  sensors,rgb_indicator}.{h,cpp}` + `ro_types.h` is **platform-independent** — it
  must NOT include `<Arduino.h>`/`<EQSP32.h>`. It reaches hardware only through
  `ISkidIO` (io_interface.h) and takes time as a `tick(now_ms)` parameter (no
  `millis()`). That is what makes it host-testable and deterministic.
- `eqsp32_io.{h,cpp}` is the ONLY core file that includes `EQSP32.h` (plus the
  `.ino`). It maps `ISkidIO` to EQSP32 pins per the `ch::` channel map.
- Host tests (`tests/`, doctest via CMake) compile only the portable core — never
  `eqsp32_io.cpp`. If the core gains an Arduino include, `pixi run test` breaks.
  `tests/support/plant_model.*` is a virtual skid for closed-loop scenario tests.
- Spec §11 setpoints live in the `Params` struct (ro_types.h); a test asserts the
  defaults match the spec. §13 open points = one-line `Params` edits.
- After editing core .cpp files, run BOTH `pixi run test` and `pixi run build`
  (the same files compile into the firmware).

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
- `firmware/firmware.ino` is a thin entry (folder name must match the `.ino`
  name); it only wires `Eqsp32SkidIO` to `RoController` and pumps `tick()`.
- API reference: top comment block of `EQSP32/src/EQSP32.h`. Pin values are
  0–1000 (= 0–100% PWM). `readPin` units: CIN = mA×100 (broken wire `<350`,
  over-current `-1`), AIN = mV, TIN = °C×10 (open `-9999` / short `9999`),
  PCC = pulse count cleared on read. CIN/AIN/TIN need pins 1–8; PCC pins 9–16.
