#!/usr/bin/env bash
# repo-status.sh — show the state of the two Quantum_42 repositories side by side.
#
# The project spans two independent git repos (see REPOS.md). They are easy to
# confuse, so this prints each one's branch, HEAD, last commit, working-tree
# state and remote, plus how the firmware submodule pin compares to the
# firmware repo's current HEAD.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FIRMWARE_DIR="${FIRMWARE_DIR:-$HOME/Documents/Arduino/Robocup.ino}"

print_repo() {
  local label="$1" dir="$2"
  echo "──────────────────────────────────────────────────────────────"
  echo "$label"
  echo "  path   : $dir"
  if [ ! -d "$dir/.git" ]; then
    echo "  ⚠️  not a git repository (missing or moved?)"
    echo
    return 0
  fi
  local branch head last remote dirty
  branch="$(git -C "$dir" branch --show-current 2>/dev/null)"
  head="$(git -C "$dir" rev-parse --short HEAD 2>/dev/null)"
  last="$(git -C "$dir" log -1 --format='%ci  %s' 2>/dev/null)"
  remote="$(git -C "$dir" remote get-url origin 2>/dev/null)"
  dirty="$(git -C "$dir" status --porcelain 2>/dev/null | wc -l | tr -d ' ')"
  echo "  branch : ${branch:-(detached)}"
  echo "  HEAD   : $head"
  echo "  last   : $last"
  echo "  remote : ${remote:-(none)}"
  if [ "$dirty" = "0" ]; then
    echo "  state  : clean"
  else
    echo "  state  : $dirty uncommitted change(s)"
  fi
  echo
}

echo "Quantum_42 repository status  —  $(date '+%Y-%m-%d %H:%M:%S')"
echo
print_repo "PROJECT repo  (specs / docs / CAD — this repo)" "$PROJECT_DIR"
print_repo "FIRMWARE repo (active Mega 2560 firmware)"       "$FIRMWARE_DIR"

if git -C "$PROJECT_DIR" config -f .gitmodules --get submodule.firmware.url >/dev/null 2>&1; then
  echo "──────────────────────────────────────────────────────────────"
  echo "Submodule pin (firmware/) vs firmware repo HEAD"
  pin="$(git -C "$PROJECT_DIR" ls-files -s firmware 2>/dev/null | awk '{print $2}')"
  fwhead="$(git -C "$FIRMWARE_DIR" rev-parse HEAD 2>/dev/null)"
  echo "  pinned  : ${pin:0:12}"
  echo "  firmware: ${fwhead:0:12}"
  if [ -n "$pin" ] && [ "$pin" = "$fwhead" ]; then
    echo "  ✅ in sync"
  else
    echo "  ⚠️  out of sync — run scripts/sync-firmware.sh (normal after firmware work)"
  fi
  echo
fi

echo "Newest code wins → compare the 'last' dates above."
