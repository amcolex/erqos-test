# Project notes for Claude

Firmware for the **Erqos EQSP32**, an ESP32-S3 industrial PLC. We build the
vendor's Arduino library with `arduino-cli`, orchestrated by **pixi**.

## Build & run
- `pixi run setup` — one-time; installs arduino-cli + ESP32 core + RadioLib into `./.arduino/` (git-ignored, self-contained; nothing global).
- `pixi run build` / `upload` / `flash` / `monitor` / `boards` / `clean`.
- Task scripts live in `scripts/`; shared config is `scripts/_env.sh`; tunables
  (versions, `FQBN`, `PORT`) are in `pixi.toml` `[activation.env]`.

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

## Editing firmware
- `firmware/firmware.ino` is the sketch (folder name must match the `.ino` name).
- API reference: top comment block of `EQSP32/src/EQSP32.h`. 36 example sketches
  in `EQSP32/examples/`. Pin values are 0–1000 (= 0–100%); `readPin` returns
  mV for AIN, °C×10 for TIN, etc.
