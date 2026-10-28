#!/usr/bin/env bash
# Run in the robot's graphical session, or pass dev / auto-rsr over SSH.
set -euo pipefail
mode="${1:-}"
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/bloom"
mkdir -p "$state_dir"
exec 9>"$state_dir/mode-selector.lock"
flock -n 9 || exit 0
if [[ -z "$mode" ]]; then
    while true; do
        mode=$(zenity --entry --title='M robot startup mode' --width=520 \
            --text=$'Choose a mode with the keyboard, then press Enter.\n\ndev — Ubuntu desktop; robot stack stays off.\nauto-rsr — face, pairing and sentence playback for dev-bloom.unr.dev.\n\nCancel leaves the desktop in development mode.' \
            --entry-text='') || mode=dev
        case "${mode,,}" in dev|auto-rsr) break ;; esac
        zenity --error --text='Enter dev or auto-rsr.' || true
    done
fi
mode="${mode,,}"
case "$mode" in dev|auto-rsr) ;; *) echo 'Usage: select_boot_mode.sh [dev|auto-rsr]' >&2; exit 2 ;; esac
systemctl --user stop bloom-auto-rsr.service
if [[ "$mode" == auto-rsr ]]; then
    for name in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS; do
        [[ -z "${!name:-}" ]] || systemctl --user import-environment "$name"
    done
    systemctl --user start bloom-auto-rsr.service
fi
printf '%s\n' "$mode" > "$state_dir/mode"
echo "M mode: $mode"
