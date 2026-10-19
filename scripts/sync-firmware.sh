#!/usr/bin/env bash
# sync-firmware.sh — re-point the firmware/ submodule pin at the firmware repo's
# current remote HEAD.
#
# The canonical firmware working copy lives in the Arduino sketchbook
# (~/Documents/Arduino/Robocup.ino); the firmware/ submodule in this repo is only
# a pinned reference. Run this AFTER you have pushed new firmware, so the project
# repo records which firmware commit it corresponds to.
#
# This does NOT push firmware and does NOT commit for you — it only updates and
# stages the submodule pointer.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Updating submodule 'firmware/' to its remote HEAD ..."
git -C "$PROJECT_DIR" submodule update --init --remote firmware

echo "Staging updated pin ..."
git -C "$PROJECT_DIR" add firmware

newpin="$(git -C "$PROJECT_DIR" ls-files -s firmware | awk '{print $2}')"
echo "firmware/ now pinned to ${newpin:0:12}"
echo
echo "Next: commit the updated pin in the project repo:"
echo "  git -C \"$PROJECT_DIR\" commit -m 'Update firmware submodule pin'"
