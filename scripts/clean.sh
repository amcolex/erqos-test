#!/usr/bin/env bash
# Remove build artifacts (keeps the downloaded toolchain & cores).
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rm -rf "$BUILD_DIR" "$PROJECT_ROOT/build"
echo "==> Removed $BUILD_DIR and $PROJECT_ROOT/build"
