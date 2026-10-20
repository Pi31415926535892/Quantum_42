#!/usr/bin/env bash
# Host-side dry-run test runner: compiles and runs test/*_test.cpp
# and test/*_test.py, aggregates results, exits nonzero on any failure.
#
# test/arduino_stub is on the include path and stub.cpp is linked into every
# suite so the .ino delivery files (which #include <Arduino.h>) can be
# type-checked and driven without an AVR toolchain. The stub is inert, so
# linking it into suites that never touch it costs nothing.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TEST_DIR="$ROOT/test"
BUILD_DIR="$TEST_DIR/.build"
STUB_DIR="$TEST_DIR/arduino_stub"
CXX="${CXX:-g++}"
CXXFLAGS="-std=c++14 -Wall -Wextra -I$TEST_DIR -I$STUB_DIR -I$ROOT/src/rescue_robot"
STUB_SRC="$STUB_DIR/stub.cpp"
# The legacy Robocup.ino includes <QTRSensors.h> by vendor name, so the stub
# directory itself must be searched, not just its parent.

mkdir -p "$BUILD_DIR"

failures=0
ran=0

echo "== host C++ tests =="

shopt -s nullglob
cpp_tests=("$TEST_DIR"/*_test.cpp)
shopt -u nullglob

if [ ${#cpp_tests[@]} -eq 0 ]; then
    echo "no *_test.cpp found"
fi

for src in "${cpp_tests[@]}"; do
    name="$(basename "$src" .cpp)"
    bin="$BUILD_DIR/$name"
    ran=$((ran + 1))
    if ! $CXX $CXXFLAGS "$src" "$STUB_SRC" -o "$bin" 2>"$BUILD_DIR/$name.compile.log"; then
        echo "COMPILE FAIL: $name"
        cat "$BUILD_DIR/$name.compile.log"
        failures=$((failures + 1))
        continue
    fi
    # Run from the repo root: bench_remap_test reads the legacy sketches by
    # their repo-root paths.
    if ! (cd "$ROOT" && "$bin"); then
        echo "FAIL: $name"
        failures=$((failures + 1))
        continue
    fi
done

echo "== python tests =="

shopt -s nullglob
py_tests=("$TEST_DIR"/*_test.py)
shopt -u nullglob

if [ ${#py_tests[@]} -eq 0 ]; then
    echo "no *_test.py found"
fi

if [ ${#py_tests[@]} -gt 0 ]; then
    pushd "$ROOT" >/dev/null
    for src in "${py_tests[@]}"; do
        ran=$((ran + 1))
        module="test.$(basename "$src" .py)"
        if ! python3 -m unittest "$module"; then
            echo "FAIL: $(basename "$src")"
            failures=$((failures + 1))
        fi
    done
    popd >/dev/null
fi

echo "== summary =="
echo "suites: $ran  failures: $failures"
if [ "$failures" -ne 0 ] || [ "$ran" -eq 0 ]; then
    exit 1
fi
exit 0
