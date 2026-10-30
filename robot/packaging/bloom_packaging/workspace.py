"""Finds ROS packages in a source tree and orders them so dependencies build first."""
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Set

_DEP_TAGS = ('depend', 'build_depend', 'buildtool_depend', 'exec_depend', 'build_export_depend', 'test_depend')


@dataclass
class Package:
    name: str
    path: str
    version: str
    deps: Set[str] = field(default_factory=set)  # ROS package names only (filtered later)


def find_packages(src: str, exclude: Set[str] = frozenset()) -> Dict[str, Package]:
    found: Dict[str, Package] = {}
    for entry in sorted(os.listdir(src)):
        path = os.path.join(src, entry)
        manifest = os.path.join(path, 'package.xml')
        if not os.path.isfile(manifest) or os.path.exists(os.path.join(path, 'COLCON_IGNORE')):
            continue
        root = ET.parse(manifest).getroot()
        name = root.findtext('name')
        if not name or name in exclude:
            continue
        deps = {(el.text or '').strip() for tag in _DEP_TAGS for el in root.findall(tag)}
        found[name] = Package(name, path, (root.findtext('version') or '').strip(), deps)
    return found


def build_order(packages: Dict[str, Package]) -> List[Package]:
    """Dependencies first. Only edges between the given packages count. Raises on a cycle."""
    internal = set(packages)
    ordered: List[Package] = []
    state: Dict[str, int] = {}  # 1 = visiting, 2 = done

    def visit(name: str, chain: List[str]) -> None:
        if state.get(name) == 2:
            return
        if state.get(name) == 1:
            raise ValueError('dependency cycle: ' + ' -> '.join(chain + [name]))
        state[name] = 1
        for dep in sorted(packages[name].deps & internal):
            visit(dep, chain + [name])
        state[name] = 2
        ordered.append(packages[name])

    for name in sorted(packages):
        visit(name, [])
    return ordered
