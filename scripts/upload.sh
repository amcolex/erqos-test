#!/usr/bin/env bash
# Flash the most recent build to a connected EQSP32.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
require_cli

PORT_RESOLVED="$(resolve_port)"
echo "==> Uploading to $PORT_RESOLVED ($FQBN)"

"$ARDUINO_CLI" upload \
  --fqbn "$FQBN" \
  --port "$PORT_RESOLVED" \
  --input-dir "$BUILD_DIR" \
  "$SKETCH_DIR"

echo "==> Upload complete. View logs with: pixi run monitor"
