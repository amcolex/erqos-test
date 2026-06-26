#!/usr/bin/env bash
# One-time setup: install a pinned arduino-cli, the ESP32 v2 Arduino core, and
# RadioLib into the project-local .arduino/ directory. Safe to re-run.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

ARDUINO_CLI_VERSION="${ARDUINO_CLI_VERSION:-1.1.1}"
ESP32_CORE_VERSION="${ESP32_CORE_VERSION:-2.0.17}"
RADIOLIB_VERSION="${RADIOLIB_VERSION:-latest}"

mkdir -p "$ARDUINO_DIR/bin" "$ARDUINO_DIRECTORIES_DATA" \
         "$ARDUINO_DIRECTORIES_USER" "$ARDUINO_DIRECTORIES_DOWNLOADS"

# 1) arduino-cli binary (pinned, downloaded into the project) -----------------
if [ -x "$ARDUINO_CLI" ] && "$ARDUINO_CLI" version | grep -q "$ARDUINO_CLI_VERSION"; then
  echo "==> arduino-cli $ARDUINO_CLI_VERSION already present."
else
  echo "==> Installing arduino-cli $ARDUINO_CLI_VERSION into .arduino/bin ..."
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | BINDIR="$ARDUINO_DIR/bin" sh -s "$ARDUINO_CLI_VERSION"
fi
"$ARDUINO_CLI" version

# 2) ESP32 Arduino core (v2 — required by EQSP32's precompiled .a) ------------
echo "==> Updating board index ..."
"$ARDUINO_CLI" core update-index
echo "==> Installing esp32:esp32@${ESP32_CORE_VERSION} (this downloads the xtensa toolchain, ~1 min) ..."
"$ARDUINO_CLI" core install "esp32:esp32@${ESP32_CORE_VERSION}"

# 3) Third-party library required by EQSP32.h ---------------------------------
if [ "$RADIOLIB_VERSION" = "latest" ]; then
  echo "==> Installing RadioLib (latest) ..."
  "$ARDUINO_CLI" lib install "RadioLib"
else
  echo "==> Installing RadioLib@${RADIOLIB_VERSION} ..."
  "$ARDUINO_CLI" lib install "RadioLib@${RADIOLIB_VERSION}"
fi

echo
echo "==> Installed cores:"
"$ARDUINO_CLI" core list
echo "==> Installed libraries:"
"$ARDUINO_CLI" lib list
echo
echo "Setup complete. The EQSP32 library is used in-place from ./EQSP32."
echo "Next: pixi run build"
