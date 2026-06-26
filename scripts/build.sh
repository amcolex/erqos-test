#!/usr/bin/env bash
# Compile firmware/ for the EQSP32 (ESP32-S3). The local ./EQSP32 clone is
# passed directly with --library, so no global install / symlink is needed.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
require_cli

echo "==> Compiling $SKETCH_DIR"
echo "    board:   $FQBN"
echo "    EQSP32:  $EQSP32_LIB"
echo

"$ARDUINO_CLI" compile \
  --fqbn "$FQBN" \
  --library "$EQSP32_LIB" \
  --build-path "$BUILD_DIR" \
  --warnings default \
  "$SKETCH_DIR"

echo
echo "==> Build OK. Artifacts in $BUILD_DIR"
echo "    Flash with: pixi run flash   (or: pixi run upload)"
