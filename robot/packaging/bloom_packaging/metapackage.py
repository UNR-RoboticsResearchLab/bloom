"""The metapackage: one version string that pins the exact version of everything on the robot.

`fleet-ota` installs `<metapackage>=<version>`; apt then pulls every pinned dependency at its exact
version, so the metapackage version *is* the robot's software state.

It also owns the robot's env file (secrets) in one limited way: its postinst creates an empty,
root-only file if none exists and never touches an existing one. The file is not part of the package
payload, so dpkg neither overwrites it on upgrade nor restores an old copy on rollback.
"""
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# Anything a shell could interpret: the path ends up inside a maintainer script.
_UNSAFE = frozenset(' \t\n\'"$`\\;&|<>*?')
DEFAULT_MAINTAINER = 'jrkre <joshuaknight@unr.edu>'


@dataclass
class MetaSpec:
    name: str
    version: str
    description: str
    pins: Dict[str, str] = field(default_factory=dict)      # package -> exact version
    extra_depends: List[str] = field(default_factory=list)  # verbatim, e.g. 'libportaudio2'
    env_files: List[str] = field(default_factory=list)      # created empty/root-only if missing
    maintainer: str = DEFAULT_MAINTAINER


def control_text(spec: MetaSpec) -> str:
    depends = [f'{name} (= {version})' for name, version in sorted(spec.pins.items())] + list(spec.extra_depends)
    lines = [
        f'Package: {spec.name}',
        f'Version: {spec.version}',
        'Section: misc',
        'Priority: optional',
        'Architecture: all',
        f'Maintainer: {spec.maintainer}',
    ]
    if depends:
        lines.append('Depends: ' + ', '.join(depends))
    lines.append(f'Description: {spec.description}')
    return '\n'.join(lines) + '\n'


def postinst_text(env_files: List[str]) -> str:
    """Honours $DPKG_ROOT (dpkg's alternate-root variable) so it can be tested without touching /etc."""
    body = ['#!/bin/sh', 'set -e', '']
    for path in env_files:
        if not path.startswith('/') or any(c in path for c in _UNSAFE):
            raise ValueError(f'unsafe env file path: {path!r}')
        body += [
            f'f="${{DPKG_ROOT:-}}{path}"',
            'if [ ! -e "$f" ]; then',
            '    mkdir -p "$(dirname "$f")"',
            '    ( umask 077; : > "$f" )',
            '    chmod 0600 "$f"',
            '    if [ "$(id -u)" = 0 ]; then chown root:root "$f"; fi',
            'fi',
        ]
    body += ['', 'exit 0', '']
    return '\n'.join(body)


def build(spec: MetaSpec, out_dir: str) -> str:
    """Builds the .deb with dpkg-deb and returns its path."""
    for tool in ('dpkg-deb',):
        if not shutil.which(tool):
            raise RuntimeError(f'{tool} is required')
    os.makedirs(out_dir, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        debian = os.path.join(tmp, 'DEBIAN')
        os.makedirs(debian)
        with open(os.path.join(debian, 'control'), 'w') as f:
            f.write(control_text(spec))
        if spec.env_files:
            post = os.path.join(debian, 'postinst')
            with open(post, 'w') as f:
                f.write(postinst_text(spec.env_files))
            os.chmod(post, 0o755)
        deb = os.path.join(out_dir, f'{spec.name}_{spec.version}_all.deb')
        subprocess.run(['dpkg-deb', '--root-owner-group', '--build', tmp, deb], check=True,
                       stdout=subprocess.DEVNULL)
    return deb


def read_deb_field(deb: str, name: str) -> Optional[str]:
    res = subprocess.run(['dpkg-deb', '-f', deb, name], capture_output=True, text=True, check=True)
    return res.stdout.strip() or None
