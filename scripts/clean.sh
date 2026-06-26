#!/usr/bin/env bash
# Remove build artifacts (keeps the downloaded toolchain & cores).
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rm -rf "$BUILD_DIR"
echo "==> Removed $BUILD_DIR"
