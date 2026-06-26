#!/usr/bin/env bash
# Local reproducible build. Mirrors the canonical src/main.cpp into the Arduino
# sketch target, then compiles with the pinned profile (sketch/sketch.yaml).
#
# Usage:  scripts/build.sh [--flash COMxx]
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

cp src/main.cpp sketch/sketch.ino
echo "==> Compiling (profile: openair)..."
arduino-cli compile --profile openair --output-dir build sketch

if [[ "${1:-}" == "--flash" && -n "${2:-}" ]]; then
  echo "==> Flashing $2 ..."
  arduino-cli upload --profile openair --port "$2" --input-dir build sketch
fi
