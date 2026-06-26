#!/usr/bin/env bash
# Publish a compiled firmware image to the AirSentinels OTA endpoint.
#
# Flow:
#   1. compile sketch/ to build/sketch.ino.bin (unless --no-build)
#   2. read FW_VERSION from src/main.cpp
#   3. compute md5 + size
#   4. upload openair-nextpm-<version>.bin + openair-nextpm.json to the VPS
#
# Deployed stations poll openair-nextpm.json every 6 h, compare "version" to
# their compiled FW_VERSION, and OTA-update when it differs.
#
# Usage:  scripts/publish-firmware.sh [--no-build]
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

SSH_KEY="$HOME/.ssh/nextpm_vps"
VPS="root@185.97.146.160"
REMOTE_DIR="/root/openair/pb_public/firmware"
FQBN="esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs,FlashSize=4M,CPUFreq=160,FlashFreq=80,FlashMode=qio"
BIN="build/sketch.ino.bin"

# src/main.cpp is the canonical source; sketch/sketch.ino is a generated mirror
# (the Arduino build target). Always re-mirror before building so the two can
# never diverge silently.
cp src/main.cpp sketch/sketch.ino

if [[ "${1:-}" != "--no-build" ]]; then
  echo "==> Compiling (pinned profile)..."
  arduino-cli compile --profile openair --output-dir build sketch
fi

[[ -f "$BIN" ]] || { echo "ERROR: $BIN not found (compile first)"; exit 1; }

# FW_VERSION from the firmware source — single source of truth.
VERSION="$(grep -oE 'FW_VERSION\[\][[:space:]]*=[[:space:]]*"[^"]+"' src/main.cpp | grep -oE '"[^"]+"' | tr -d '"')"
[[ -n "$VERSION" ]] || { echo "ERROR: could not parse FW_VERSION"; exit 1; }

MD5="$(md5sum "$BIN" | cut -d' ' -f1)"
SIZE="$(wc -c < "$BIN" | tr -d ' ')"
BIN_NAME="openair-nextpm-${VERSION}.bin"
URL="https://station.airsentinels.fr/firmware/${BIN_NAME}"

echo "==> Version : $VERSION"
echo "==> Size    : $SIZE bytes"
echo "==> MD5     : $MD5"
echo "==> URL     : $URL"

MANIFEST="$(mktemp)"
cat > "$MANIFEST" <<EOF
{
  "version": "${VERSION}",
  "url": "${URL}",
  "md5": "${MD5}",
  "size": ${SIZE}
}
EOF

echo "==> Uploading binary + manifest..."
ssh -i "$SSH_KEY" -o IdentitiesOnly=yes "$VPS" "mkdir -p $REMOTE_DIR"
scp -i "$SSH_KEY" -o IdentitiesOnly=yes "$BIN" "$VPS:$REMOTE_DIR/$BIN_NAME"
# Upload manifest LAST and atomically (temp then mv) so a station never reads a
# manifest pointing at a binary that isn't there yet.
scp -i "$SSH_KEY" -o IdentitiesOnly=yes "$MANIFEST" "$VPS:$REMOTE_DIR/.openair-nextpm.json.tmp"
ssh -i "$SSH_KEY" -o IdentitiesOnly=yes "$VPS" "mv $REMOTE_DIR/.openair-nextpm.json.tmp $REMOTE_DIR/openair-nextpm.json"
rm -f "$MANIFEST"

echo "==> Done. Manifest live at https://station.airsentinels.fr/firmware/openair-nextpm.json"
