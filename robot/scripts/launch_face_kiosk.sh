#!/usr/bin/env bash
# Serves the built vizij face app locally and opens it full-screen in a
# kiosk-mode Chromium window, pointed at this robot's own robotId.
#
# Companion to bloom_face_bridge (ROS topics -> local WebSocket/HTTP) and
# rsr_speech_node (RSR sentence playback) -- run alongside those, not instead
# of them. Does not touch bloom_face/face_node.py; run both side by side
# until the vizij face is validated on real hardware.
set -euo pipefail

ROBOT_CFG="${HOME}/.bloom/robot.cfg"
FACE_APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../kiosk-face/apps/tutorial-fullscreen-face" && pwd)"
PREVIEW_PORT="${FACE_KIOSK_PORT:-4173}"

if [[ ! -f "$ROBOT_CFG" ]]; then
    echo "error: $ROBOT_CFG not found -- robot must be registered first (see bloom_node)" >&2
    exit 1
fi

robot_id="$(grep -E '^robot_id=' "$ROBOT_CFG" | tail -1 | cut -d= -f2- | xargs)"
if [[ -z "$robot_id" ]]; then
    echo "error: robot_id not set in $ROBOT_CFG yet -- robot must complete registration first" >&2
    exit 1
fi

echo "Building face app..."
(cd "$FACE_APP_DIR" && pnpm run build)

echo "Serving built app on http://localhost:${PREVIEW_PORT}/ ..."
(cd "$FACE_APP_DIR" && pnpm exec vite preview --port "$PREVIEW_PORT" --strictPort) &
PREVIEW_PID=$!
trap 'kill "$PREVIEW_PID" 2>/dev/null || true' EXIT

# Give the preview server a moment to come up before pointing chromium at it.
sleep 2

FACE_URL="http://localhost:${PREVIEW_PORT}/?robotId=${robot_id}"
echo "Launching kiosk browser at ${FACE_URL}"
chromium \
    --kiosk \
    --noerrdialogs \
    --disable-infobars \
    --no-first-run \
    --disable-session-crashed-bubble \
    --disable-restore-session-state \
    --disable-features=TranslateUI \
    --overscroll-history-navigation=0 \
    --no-default-browser-check \
    "$FACE_URL"
