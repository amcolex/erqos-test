# erqos-firmware

Firmware for the **Erqos EQSP32** — an ESP32-S3-based industrial IoT / micro-PLC
controller. The dev environment is fully managed by [pixi](https://pixi.sh): a
single `pixi run setup` pulls down a pinned, self-contained Arduino toolchain so
nothing touches your global Arduino install.

## What's here

```
.
├── pixi.toml            # environment + task definitions (start here)
├── firmware/
│   └── firmware.ino     # the sketch you edit — a working EQSP32 starter
├── EQSP32/              # the vendor library (cloned from Erqos/EQSP32), used in place
├── scripts/             # task implementations invoked by pixi
└── .arduino/            # downloaded toolchain, cores & build output (git-ignored)
```

## Toolchain

This is an **Arduino library project**, so we build it the way Erqos ships it —
with `arduino-cli` and the official ESP32 Arduino core, driven by pixi.

| Component        | Version  | Why pinned |
|------------------|----------|------------|
| arduino-cli      | 1.1.1    | reproducible CLI |
| esp32 Arduino core | **2.0.17** | EQSP32's `libEQSP32.a` is precompiled against the **v2** core. Do **not** move to 3.x — Erqos states the library "supports up to esp32 v2 core" and the ABI would break. |
| RadioLib         | 7.7.1    | only third-party header `EQSP32.h` includes (LoRa) |
| Board (FQBN)     | `esp32:esp32:esp32s3` | ESP32S3 Dev Module |
| Flash / Partition | `8M` / `8M with spiffs` (`default_8MB`) | per Erqos quickstart guide |

Everything lives under `./.arduino/` — delete that folder to start clean.

## Quick start

```bash
pixi run setup      # one-time: fetch arduino-cli, ESP32 core 2.0.17, RadioLib (~250 MB)
pixi run build      # compile firmware/
# plug the EQSP32 in over USB, then:
pixi run flash      # build + upload
pixi run monitor    # serial console @ 115200 baud
```

## Tasks

| Command            | Does |
|--------------------|------|
| `pixi run setup`   | Install the toolchain into `.arduino/` (idempotent). |
| `pixi run build`   | Compile `firmware/` for the EQSP32. |
| `pixi run upload`  | Flash the last build (auto-detects the port, or set `PORT`). |
| `pixi run flash`   | `build` then `upload`. |
| `pixi run monitor` | Open the serial monitor at 115200 baud. |
| `pixi run boards`  | List connected boards / serial ports. |
| `pixi run clean`   | Remove build artifacts. |

### Choosing the serial port

Port auto-detection picks the only connected board. If you have several, set it:

```bash
PORT=/dev/cu.usbserial-XXXX pixi run flash
```

(Find the name with `pixi run boards`.) You can also change the default `PORT`,
`FQBN`, and pinned versions in the `[activation.env]` block of `pixi.toml`.

## Writing firmware

Edit [`firmware/firmware.ino`](firmware/firmware.ino). The starter wires up a
digital input, a relay, a PWM output, the buzzer and the user button — see the
EQSP32 API summary at the top of [`EQSP32/src/EQSP32.h`](EQSP32/src/EQSP32.h) and
the 36 reference sketches in [`EQSP32/examples/`](EQSP32/examples/).

To try an example instead, point a build at it:

```bash
pixi run -- bash -c '"$ARDUINO_CLI" compile --fqbn "$FQBN" --library "$EQSP32_LIB" EQSP32/examples/EQSP32_CAN-Bus_Demo'
```

> Note: most examples need real I/O wired up and (for WiFi/MQTT/BLE demos)
> provisioning via the **EQConnect** mobile app; they compile regardless.

## References

- EQSP32 library & examples: <https://github.com/Erqos/EQSP32>
- EQSP32 quickstart guide: <https://erqos.com/resources/quickstart-guide/>
