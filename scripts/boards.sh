#!/usr/bin/env bash
# List connected boards / serial ports.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
require_cli
"$ARDUINO_CLI" board list
