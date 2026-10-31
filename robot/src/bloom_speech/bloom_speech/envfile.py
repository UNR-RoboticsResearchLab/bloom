"""Credentials for the speech nodes come from the process environment, not from the package.

On a robot, systemd loads the env file (`EnvironmentFile=`) before the node starts, so the nodes
need nothing more. For runs outside systemd (a shell, `ros2 launch` on a dev machine) `load_env()`
reads the same file itself. Packages never ship this file: it is created at provisioning, belongs
to the robot rather than to any package version, and so survives upgrades and rollbacks untouched.

Lookup order (existing environment variables always win, so systemd's values are never replaced):
    1. $BLOOM_ENV_FILE, if set     (e.g. BLOOM_ENV_FILE=robot/.env on a dev machine)
    2. /etc/bloom/robot.env
"""
import os
from typing import Dict, Iterable, List, Optional

DEFAULT_ENV_FILE = '/etc/bloom/robot.env'


def parse_env(lines: Iterable[str]) -> Dict[str, str]:
    """KEY=VALUE lines; blank lines and # comments ignored; one pair of matching quotes stripped
    (as systemd does)."""
    values: Dict[str, str] = {}
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        key, value = key.strip(), value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ('"', "'"):
            value = value[1:-1]
        if key:
            values[key] = value
    return values


def load_env(environ: Optional[dict] = None, paths: Optional[List[str]] = None) -> List[str]:
    """Adds missing variables from the env files to `environ` (default os.environ).

    Returns the files that were read. An unreadable file (missing, or root-only when the node runs
    as a normal user) is skipped, not an error: under systemd the variables are already there.
    """
    if environ is None:
        environ = os.environ
    if paths is None:
        paths = [p for p in (environ.get('BLOOM_ENV_FILE', ''), DEFAULT_ENV_FILE) if p]

    loaded = []
    for path in paths:
        try:
            with open(path) as f:
                values = parse_env(f)
        except OSError:
            continue
        for key, value in values.items():
            environ.setdefault(key, value)
        loaded.append(path)
    return loaded
