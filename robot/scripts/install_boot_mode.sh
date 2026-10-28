#!/usr/bin/env bash
# Install as the desktop/robot user. No root access needed.
set -euo pipefail
scripts="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "$scripts/.." && pwd)"
command -v zenity >/dev/null
command -v flock >/dev/null
mkdir -p "$HOME/.config/systemd/user" "$HOME/.config/autostart" "$HOME/.local/share/applications"
cat > "$HOME/.config/systemd/user/bloom-auto-rsr.service" <<EOF
[Unit]
Description=M AutoRSR robot stack (dev-bloom.unr.dev)
After=graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
WorkingDirectory=$workspace
ExecStart=/bin/bash "$scripts/run_auto_rsr.sh"
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=20
Restart=on-failure
RestartSec=5
EOF
cat > "$HOME/.config/autostart/bloom-mode-selector.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=M robot mode
Comment=Choose development or AutoRSR mode
Exec=/bin/bash "$scripts/select_boot_mode.sh"
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
cp "$HOME/.config/autostart/bloom-mode-selector.desktop" "$HOME/.local/share/applications/bloom-mode-selector.desktop"
systemctl --user daemon-reload
echo 'Installed. The selector opens at graphical login; the robot service waits for your choice.'
