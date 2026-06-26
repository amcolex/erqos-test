#!/usr/bin/env bash
# Open a serial monitor to the EQSP32 at 115200 baud (Ctrl-C to quit).
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
require_cli

PORT_RESOLVED="$(resolve_port)"
echo "==> Monitoring $PORT_RESOLVED at 115200 baud (Ctrl-C to exit)"

"$ARDUINO_CLI" monitor \
  --port "$PORT_RESOLVED" \
  --config baudrate=115200
