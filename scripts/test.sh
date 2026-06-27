#!/usr/bin/env bash
# Build and run the host-side unit + scenario test suite (no hardware needed).
# Uses the conda-forge C++ toolchain provided by pixi. With --coverage, also
# emits a line/branch coverage report for the control core.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

COVERAGE=0
[ "${1:-}" = "--coverage" ] && COVERAGE=1

BUILD="$PROJECT_ROOT/build/tests"

CMAKE_ARGS=(-S "$PROJECT_ROOT/tests" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Debug)
[ "$COVERAGE" = 1 ] && CMAKE_ARGS+=(-DRO_COVERAGE=ON)

echo "==> Configuring tests ($([ "$COVERAGE" = 1 ] && echo 'with coverage' || echo 'plain'))"
cmake "${CMAKE_ARGS[@]}"

echo "==> Building tests"
cmake --build "$BUILD"

# Reset accumulated coverage counters so each report reflects only this run.
[ "$COVERAGE" = 1 ] && find "$BUILD" -name '*.gcda' -delete

echo "==> Running tests"
"$BUILD/ro_tests"

if [ "$COVERAGE" = 1 ]; then
  echo
  echo "==> Coverage (control core)"
  # clang emits LLVM coverage; gcovr drives it via 'llvm-cov gcov'. Best-effort.
  gcovr --root "$PROJECT_ROOT" \
        --filter "$PROJECT_ROOT/firmware/" \
        --gcov-executable "llvm-cov gcov" \
        --print-summary "$BUILD" \
    || gcovr --root "$PROJECT_ROOT" --filter "$PROJECT_ROOT/firmware/" --print-summary "$BUILD" \
    || echo "   (coverage report skipped — gcov backend unavailable)"
fi
