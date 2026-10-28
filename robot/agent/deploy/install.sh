#!/usr/bin/env bash
# Installs or upgrades the Bloom robot agent on an Ubuntu (arm64) robot. Idempotent.
# Run from an extracted robot-agent-<version>.tar.gz:
#
#   sudo ./install.sh --backend-url https://bloom.example.edu --robot-id <guid> --secret-file /path/to/secret
#   sudo ./install.sh                     # upgrade: keeps existing config and secret
#   sudo ./install.sh --disable-sshd      # also stop sshd -- only after the agent is confirmed working
#
# Get the robot id + secret from an admin: POST /api/robot/register (new robot) or
# POST /api/robot/{id}/credentials (rotate). The secret can also be piped on stdin with --secret-file -.

set -euo pipefail

USER_NAME=bloom-ops
PREFIX=/opt/bloom-agent
CONFIG_DIR=/etc/bloom-agent
UNIT=/etc/systemd/system/bloom-agent.service
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BACKEND_URL=""
ROBOT_ID=""
SECRET_FILE=""
DISABLE_SSHD=false

while [ $# -gt 0 ]; do
  case "$1" in
    --backend-url) BACKEND_URL="$2"; shift 2 ;;
    --robot-id) ROBOT_ID="$2"; shift 2 ;;
    --secret-file) SECRET_FILE="$2"; shift 2 ;;
    --disable-sshd) DISABLE_SSHD=true; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 64 ;;
  esac
done

[ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)" >&2; exit 1; }
[ -x "$HERE/robot-agent" ] || { echo "robot-agent binary not found next to install.sh" >&2; exit 1; }

VERSION="$("$HERE/robot-agent" version | cut -d' ' -f1)"
echo "Installing robot-agent $VERSION"

# ── Dedicated non-root user with passwordless sudo ───────────────────────────
# Admins' shells run as this user. Its password is locked: the only way in is the agent tunnel.
if ! id "$USER_NAME" >/dev/null 2>&1; then
  useradd --create-home --shell /bin/bash --comment "Bloom fleet remote shell" "$USER_NAME"
  passwd --lock "$USER_NAME" >/dev/null
fi
SUDOERS_TMP="$(mktemp)"
echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > "$SUDOERS_TMP"
visudo -cf "$SUDOERS_TMP" >/dev/null
install -m 0440 -o root -g root "$SUDOERS_TMP" "/etc/sudoers.d/$USER_NAME"
rm -f "$SUDOERS_TMP"

# ── Binary: versioned directory + "current" symlink, so an upgrade (and Phase 2 OTA) is an
#    atomic symlink swap with the previous version still on disk for rollback. ──
install -d -m 0755 "$PREFIX/$VERSION"
install -m 0755 "$HERE/robot-agent" "$PREFIX/$VERSION/robot-agent"
[ -f "$HERE/README.md" ] && install -m 0644 "$HERE/README.md" "$PREFIX/$VERSION/README.md"
ln -sfn "$PREFIX/$VERSION" "$PREFIX/current.new"
mv -T "$PREFIX/current.new" "$PREFIX/current"

# ── Config and secret ────────────────────────────────────────────────────────
install -d -m 0750 -o root -g "$USER_NAME" "$CONFIG_DIR"
install -m 0644 "$HERE/agent.json.example" "$CONFIG_DIR/agent.json.example"
if [ -n "$BACKEND_URL" ] || [ -n "$ROBOT_ID" ] || [ ! -f "$CONFIG_DIR/agent.json" ]; then
  [ -n "$BACKEND_URL" ] && [ -n "$ROBOT_ID" ] || {
    echo "first install needs --backend-url and --robot-id" >&2; exit 64; }
  python3 - "$CONFIG_DIR/agent.json" "$BACKEND_URL" "$ROBOT_ID" <<'PY'
import json, os, sys
path, url, robot_id = sys.argv[1:4]
cfg = json.load(open(path)) if os.path.exists(path) else json.load(open(path + ".example"))
cfg.update(backend_url=url, robot_id=robot_id)
json.dump(cfg, open(path, "w"), indent=2)
PY
  chown root:"$USER_NAME" "$CONFIG_DIR/agent.json"
  chmod 0640 "$CONFIG_DIR/agent.json"
fi
if [ -n "$SECRET_FILE" ]; then
  # Owned by the agent's user: it re-reads the file before each login, so rotating is just
  # rewriting it (sudo tee /etc/bloom-agent/secret) -- no restart needed.
  umask 077
  if [ "$SECRET_FILE" = "-" ]; then cat > "$CONFIG_DIR/secret"; else cp "$SECRET_FILE" "$CONFIG_DIR/secret"; fi
  chown "$USER_NAME:$USER_NAME" "$CONFIG_DIR/secret"
  chmod 0600 "$CONFIG_DIR/secret"
fi
[ -s "$CONFIG_DIR/secret" ] || { echo "no secret installed: pass --secret-file" >&2; exit 64; }

# ── Service ──────────────────────────────────────────────────────────────────
install -m 0644 "$HERE/bloom-agent.service" "$UNIT"
systemctl daemon-reload
systemctl enable bloom-agent.service >/dev/null
systemctl restart bloom-agent.service

echo
sudo -u "$USER_NAME" env $(grep -E '^(HTTPS?_PROXY|https?_proxy|NO_PROXY|no_proxy)=' /etc/environment 2>/dev/null | tr -d '"' || true) \
  "$PREFIX/current/robot-agent" doctor --config "$CONFIG_DIR/agent.json" && DOCTOR_OK=true || DOCTOR_OK=false

# ── Optionally close sshd: the agent becomes the only way in ─────────────────
if [ "$DISABLE_SSHD" = true ]; then
  if [ "$DOCTOR_OK" != true ]; then
    echo "NOT disabling sshd: doctor reported failures, and you'd have no way back in." >&2
    exit 1
  fi
  systemctl disable --now ssh.socket ssh.service 2>/dev/null || true
  echo "sshd disabled. Re-enable locally with: sudo systemctl enable --now ssh"
fi

echo
echo "robot-agent $VERSION installed. Logs: journalctl -u bloom-agent -f"
