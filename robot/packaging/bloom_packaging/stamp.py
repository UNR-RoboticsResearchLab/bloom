"""Rewrites what `bloom-generate rosdebian` produces so every package carries the build version.

bloom takes the version from package.xml (0.0.1, 0.1.0, ...) and those never change, so a code change
would ship under the same version and `apt` would see nothing to upgrade. Everything here is pure text
handling so it can be tested without building anything.
"""
import re
from typing import Dict, Iterable, List

_CHANGELOG_FIRST = re.compile(r'^(?P<name>\S+) \((?P<version>[^)]+)\)(?P<rest>.*)$')


def deb_name(ros_distro: str, package: str) -> str:
    return f'ros-{ros_distro}-{package.replace("_", "-")}'


def stamp_changelog(text: str, version: str) -> str:
    """Replaces the version on the first line: `ros-jazzy-x (0.0.1-0noble) noble; ...`."""
    first, sep, rest = text.partition('\n')
    m = _CHANGELOG_FIRST.match(first)
    if not m:
        raise ValueError(f'unrecognised changelog header: {first!r}')
    return f'{m.group("name")} ({version}){m.group("rest")}{sep}{rest}'


def _split_stanzas(text: str) -> List[List[str]]:
    stanzas, current = [], []
    for line in text.split('\n'):
        if line.strip() == '':
            if current:
                stanzas.append(current)
                current = []
        else:
            current.append(line)
    if current:
        stanzas.append(current)
    return stanzas


def _fields(stanza: List[str]) -> List[List[str]]:
    """[[field-name, full value including continuation lines], ...] preserving order."""
    fields: List[List[str]] = []
    for line in stanza:
        if line[0] in ' \t' and fields:
            fields[-1][1] += '\n' + line
        else:
            name, _, value = line.partition(':')
            fields.append([name, value.strip()])
    return fields


def _pin(value: str, internal: Dict[str, str]) -> str:
    items = []
    for item in value.replace('\n', ' ').split(','):
        item = item.strip()
        if not item:
            continue
        name = item.split()[0]
        if name in internal and '(' not in item:
            item = f'{name} (= {internal[name]})'
        items.append(item)
    return ', '.join(items)


def pin_internal_depends(control: str, internal: Iterable[str], version: str) -> str:
    """Turns bare references to our own packages in (Build-)Depends into exact `(= version)` pins.

    Packages from one build always travel together, so a robot can never end up with, say, a new
    bloom-node against an old bloom-msgs.
    """
    pins = {name: version for name in internal}
    out = []
    for stanza in _split_stanzas(control):
        lines = []
        for name, value in _fields(stanza):
            if name in ('Depends', 'Build-Depends', 'Pre-Depends', 'Recommends'):
                value = _pin(value, pins)
            lines.append(f'{name}: {value}')
        out.append('\n'.join(lines))
    return '\n\n'.join(out) + '\n'


def local_rosdep_yaml(mapping: Dict[str, str], os_name: str, codename: str) -> str:
    """rosdep rules mapping our own ROS package names to the deb names we are about to build, so
    packages that depend on each other resolve without publishing anything first."""
    lines = []
    for package, deb in sorted(mapping.items()):
        lines += [f'{package}:', f'  {os_name}:', f'    {codename}: [{deb}]']
    return '\n'.join(lines) + '\n'
