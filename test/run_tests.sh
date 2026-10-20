#!/usr/bin/env bash
# Host-side tests for the Nicla Vision scripts in nicla/.
#
# The Mega firmware and its test suite live in the firmware repo
# (~/Documents/Arduino/Robocup.ino) — see REPOS.md. This runner only covers
# the code that still lives in this repo.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

shopt -s nullglob
py_tests=(test/*_test.py)
shopt -u nullglob

if [ ${#py_tests[@]} -eq 0 ]; then
  echo "no *_test.py found"
  exit 1
fi

failures=0
ran=0

for src in "${py_tests[@]}"; do
  ran=$((ran + 1))
  module="test.$(basename "$src" .py)"
  if ! python3 -m unittest "$module"; then
    echo "FAIL: $(basename "$src")"
    failures=$((failures + 1))
  fi
done

echo "== summary =="
echo "suites: $ran  failures: $failures"
[ "$failures" -eq 0 ]
