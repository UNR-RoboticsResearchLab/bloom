#!/usr/bin/env bash
# Applies robot credentials issued by the Bloom dashboard to this robot. Run as root.
#
#   sudo ./provision.sh bloom-robot.provision.json           # the file downloaded from the dashboard
#   sudo ./provision.sh FILE --remove                         # ...and securely delete it afterwards
#   echo "$SECRET" | sudo ./provision.sh --secret-stdin       # secret only (what a dashboard "push" does)
#
# It updates, all-or-nothing (any failure rolls every file back to how it was):
#   /etc/bloom-agent/secret     the remote-shell agent's secret
#   /etc/bloom-agent/agent.json backend_url + robot_id (file mode only)
#   bloom_node's .env           BLOOM_ROBOT_ID / BLOOM_ROBOT_SECRET; found from the EnvironmentFile= line of
#                               /etc/systemd/system/bloom.service, or given with --env-file (repeatable)
# then restarts bloom-agent and bloom (bloom_node) so they log in with the new secret.
#
# Options:
#   --env-file PATH       bloom_node env file to update (default: from bloom.service)
#   --no-restart-agent    don't restart bloom-agent (a dashboard push does this: the agent runs this script
#                         itself, and the backend drops its tunnel so it reconnects with the new secret)
#   --no-restart-node     don't restart bloom.service
#   --config-dir DIR      default /etc/bloom-agent
#   --remove              delete the provisioning file when done
#
# The provisioning file holds the robot's secret in plain text. Treat it like a password.

set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "provision.sh must run as root (sudo)" >&2; exit 1; }
command -v python3 >/dev/null || { echo "python3 is required" >&2; exit 1; }

# The program is on fd 3 so that stdin stays free for --secret-stdin.
exec python3 /dev/fd/3 "$@" 3<<'PY'
import argparse
import grp
import json
import os
import pwd
import re
import shutil
import subprocess
import sys
import tempfile

AGENT_USER = "bloom-ops"
SECRET_RE = re.compile(r"^[A-Za-z0-9_-]{16,512}$")
GUID_RE = re.compile(r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$")
BLOOM_UNIT = "/etc/systemd/system/bloom.service"
AGENT_UNIT = "/etc/systemd/system/bloom-agent.service"


def die(message):
    print(message, file=sys.stderr)
    sys.exit(1)


def lookup(name, getter, attr):
    try:
        return getattr(getter(name), attr)
    except KeyError:
        return None


class Files:
    """Atomic writes that remember what was there, so everything can be put back."""

    def __init__(self):
        self.original = {}  # path -> (bytes, mode, uid, gid) or None if it didn't exist
        self.written = []

    def write(self, path, data, mode, uid=None, gid=None):
        directory = os.path.dirname(path) or "."
        if path not in self.original:
            try:
                st = os.stat(path)
                with open(path, "rb") as f:
                    self.original[path] = (f.read(), st.st_mode & 0o7777, st.st_uid, st.st_gid)
            except FileNotFoundError:
                self.original[path] = None
        if os.path.exists(path):
            st = os.stat(path)
            mode, uid, gid = st.st_mode & 0o7777, st.st_uid, st.st_gid
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, prefix=".provision-")
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
            os.chmod(tmp, mode)
            if uid is not None:
                os.chown(tmp, uid, gid if gid is not None else -1)
            os.replace(tmp, path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise
        self.written.append(path)

    def rollback(self):
        for path in reversed(self.written):
            before = self.original.get(path)
            try:
                if before is None:
                    os.unlink(path)
                else:
                    data, mode, uid, gid = before
                    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".provision-")
                    with os.fdopen(fd, "wb") as f:
                        f.write(data)
                    os.chmod(tmp, mode)
                    os.chown(tmp, uid, gid)
                    os.replace(tmp, path)
            except OSError as e:
                print(f"WARNING: could not restore {path}: {e}", file=sys.stderr)


def set_env_keys(text, values):
    """Replace KEY=... lines (or append them), leaving every other line untouched."""
    lines = text.splitlines()
    remaining = dict(values)
    for i, line in enumerate(lines):
        m = re.match(r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)=", line)
        if m and m.group(1) in remaining:
            lines[i] = f"{m.group(1)}={remaining.pop(m.group(1))}"
    lines += [f"{k}={v}" for k, v in remaining.items()]
    return "\n".join(lines) + "\n"


def detect_env_files():
    found = []
    try:
        with open(BLOOM_UNIT) as f:
            for line in f:
                m = re.match(r"^\s*EnvironmentFile=-?(\S+)\s*$", line)
                if m:
                    found.append(m.group(1))
    except OSError:
        pass
    return found


def unit_exists(path):
    return os.path.exists(path)


def systemctl(*args):
    return subprocess.run(["systemctl", *args], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("file", nargs="?")
    ap.add_argument("--secret-stdin", action="store_true")
    ap.add_argument("--env-file", action="append", default=[])
    ap.add_argument("--no-restart-agent", action="store_true")
    ap.add_argument("--no-restart-node", action="store_true")
    ap.add_argument("--config-dir", default="/etc/bloom-agent")
    ap.add_argument("--remove", action="store_true")
    ap.add_argument("-h", "--help", action="store_true")
    args = ap.parse_args()
    if args.help:
        print("See the comment at the top of provision.sh for usage.")
        return

    # ── Read and validate the credentials ────────────────────────────────────
    robot_id = backend_url = None
    if args.secret_stdin:
        if args.file:
            die("give either a provisioning file or --secret-stdin, not both")
        secret = sys.stdin.readline().strip()
    else:
        if not args.file:
            die("usage: provision.sh FILE   |   provision.sh --secret-stdin   (see the header for options)")
        try:
            with open(args.file) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            die(f"cannot read provisioning file {args.file}: {e}")
        if not isinstance(data, dict) or data.get("version") != 1:
            die("not a Bloom provisioning file (expected version 1)")
        robot_id, backend_url, secret = data.get("robot_id"), data.get("backend_url"), data.get("secret")
        if not isinstance(robot_id, str) or not GUID_RE.match(robot_id):
            die("provisioning file has no valid robot_id")
        if not isinstance(backend_url, str) or not backend_url.startswith("https://"):
            die("provisioning file has no https:// backend_url")
    if not isinstance(secret, str) or not SECRET_RE.match(secret):
        die("the secret is missing or malformed")

    config_dir = args.config_dir
    agent_present = os.path.isdir(config_dir)
    env_files = args.env_file or detect_env_files()
    if not agent_present and not env_files:
        die(f"nothing to provision: {config_dir} doesn't exist (install the agent first) and no bloom_node "
            "env file was found (pass --env-file)")

    # ── Write everything, or nothing ─────────────────────────────────────────
    files = Files()
    updated = []
    agent_uid = lookup(AGENT_USER, pwd.getpwnam, "pw_uid")
    agent_gid = lookup(AGENT_USER, grp.getgrnam, "gr_gid")
    try:
        for env_file in env_files:
            try:
                with open(env_file) as f:
                    text = f.read()
            except FileNotFoundError:
                text = ""
            values = {"BLOOM_ROBOT_SECRET": secret}
            if robot_id:
                values["BLOOM_ROBOT_ID"] = robot_id
            files.write(env_file, set_env_keys(text, values).encode(), 0o600)
            updated.append(env_file)

        if agent_present:
            secret_path = os.path.join(config_dir, "secret")
            files.write(secret_path, (secret + "\n").encode(), 0o600, agent_uid, agent_gid)
            updated.append(secret_path)

            if robot_id:
                cfg_path = os.path.join(config_dir, "agent.json")
                try:
                    with open(cfg_path) as f:
                        cfg = json.load(f)
                except FileNotFoundError:
                    cfg = {}
                except ValueError as e:
                    raise RuntimeError(f"{cfg_path} is not valid JSON ({e})")
                if cfg.get("robot_id") not in (None, "", robot_id):
                    print(f"note: robot_id changes from {cfg['robot_id']} to {robot_id}")
                cfg.update(backend_url=backend_url, robot_id=robot_id)
                files.write(cfg_path, (json.dumps(cfg, indent=2) + "\n").encode(), 0o640, 0,
                            lookup(AGENT_USER, grp.getgrnam, "gr_gid"))
                updated.append(cfg_path)
    except Exception as e:  # noqa: BLE001 - any failure must undo the earlier writes
        files.rollback()
        die(f"could not apply credentials, nothing was changed: {e}")

    for path in updated:
        print(f"updated {path}")

    # ── Restart whatever uses the secret ─────────────────────────────────────
    if env_files and not args.no_restart_node and unit_exists(BLOOM_UNIT):
        r = systemctl("restart", "--no-block", "bloom.service")
        print("restarted bloom.service" if r.returncode == 0 else f"WARNING: could not restart bloom.service: {r.stderr.strip()}")
    if agent_present and not args.no_restart_agent and unit_exists(AGENT_UNIT):
        r = systemctl("restart", "bloom-agent.service")
        print("restarted bloom-agent.service" if r.returncode == 0 else f"WARNING: could not restart bloom-agent.service: {r.stderr.strip()}")

    if args.remove and args.file:
        if shutil.which("shred"):
            subprocess.run(["shred", "-u", args.file], check=False)
        if os.path.exists(args.file):
            os.unlink(args.file)
        print(f"removed {args.file}")

    print("credentials applied" + (f" (robot {robot_id})" if robot_id else ""))


main()
PY
