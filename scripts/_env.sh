# Shared environment for all EQSP32 build tasks. Sourced by the other scripts.
# Keeps the whole Arduino toolchain self-contained under .arduino/ so the repo
# is reproducible and nothing leaks into the user's global Arduino install.

set -euo pipefail

# Resolve the project root whether we're run via `pixi run` or directly.
if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
  PROJECT_ROOT="$PIXI_PROJECT_ROOT"
else
  PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi
export PROJECT_ROOT

# Self-contained Arduino home.
export ARDUINO_DIR="$PROJECT_ROOT/.arduino"
export ARDUINO_CLI="$ARDUINO_DIR/bin/arduino-cli"

# These ARDUINO_* vars override arduino-cli's config (no global config file).
export ARDUINO_DIRECTORIES_DATA="$ARDUINO_DIR/data"
export ARDUINO_DIRECTORIES_USER="$ARDUINO_DIR/user"
export ARDUINO_DIRECTORIES_DOWNLOADS="$ARDUINO_DIR/downloads"
export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

# Project layout / board config (FQBN comes from pixi.toml [activation.env]).
export SKETCH_DIR="$PROJECT_ROOT/firmware"
export EQSP32_LIB="$PROJECT_ROOT/EQSP32"
export BUILD_DIR="$ARDUINO_DIR/build"
export FQBN="${FQBN:-esp32:esp32:esp32s3:FlashSize=8M,PartitionScheme=default_8MB}"

# Fail early with a friendly message if setup hasn't run yet.
require_cli() {
  if [ ! -x "$ARDUINO_CLI" ]; then
    echo "arduino-cli not found at $ARDUINO_CLI" >&2
    echo "Run 'pixi run setup' first." >&2
    exit 1
  fi
}

# Resolve the serial port: honour $PORT, otherwise auto-detect a single board.
resolve_port() {
  if [ -n "${PORT:-}" ] && [ "${PORT}" != "auto" ]; then
    echo "$PORT"
    return
  fi
  local detected
  detected="$("$ARDUINO_CLI" board list --format jsonmini 2>/dev/null \
    | grep -o '"address":"[^"]*"' | head -n1 | cut -d'"' -f4 || true)"
  if [ -z "$detected" ]; then
    echo "No serial port detected. Plug in the EQSP32, or set PORT=/dev/..." >&2
    echo "  e.g.  PORT=/dev/cu.usbserial-XXXX pixi run upload" >&2
    exit 1
  fi
  echo "$detected"
}
