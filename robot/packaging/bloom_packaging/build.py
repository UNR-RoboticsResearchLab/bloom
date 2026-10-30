"""Builds every ROS package under a source tree into a .deb stamped with one version.

Real builds only happen with execute=True. They need debhelper, bloom, rosdep and root (they install
each finished package so the next can build against it), so they belong in a CI container, not on a
developer machine: by default this only prints the plan.
"""
import glob
import os
import shutil
import subprocess
import tempfile
from typing import Dict, List, Set

from . import stamp, workspace


def plan(src: str, ros_distro: str, version: str, exclude: Set[str]) -> List[dict]:
    packages = workspace.find_packages(src, exclude)
    return [{'package': p.name, 'path': p.path, 'deb': stamp.deb_name(ros_distro, p.name), 'version': version,
             'depends_on': sorted(p.deps & set(packages))}
            for p in workspace.build_order(packages)]


def _run(cmd: List[str], cwd: str, env: Dict[str, str]) -> None:
    print('+', ' '.join(cmd), f'   (in {cwd})', flush=True)
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def execute(src: str, out_dir: str, ros_distro: str, os_name: str, codename: str, version: str,
            exclude: Set[str], install_built: bool) -> List[str]:
    for tool in ('bloom-generate', 'dpkg-buildpackage', 'rosdep', 'git'):
        if not shutil.which(tool):
            raise RuntimeError(f'{tool} is required for a real build')

    steps = plan(src, ros_distro, version, exclude)
    internal = [s['deb'] for s in steps]
    os.makedirs(out_dir, exist_ok=True)
    built: List[str] = []

    with tempfile.TemporaryDirectory() as tmp:
        # Resolve our own package names to the debs being built, without touching /etc/ros.
        sources = os.path.join(tmp, 'rosdep-sources')
        os.makedirs(sources)
        local = os.path.join(tmp, 'local.yaml')
        with open(local, 'w') as f:
            f.write(stamp.local_rosdep_yaml({s['package']: s['deb'] for s in steps}, os_name, codename))
        shutil.copy('/etc/ros/rosdep/sources.list.d/20-default.list', os.path.join(sources, '20-default.list'))
        with open(os.path.join(sources, '10-local.list'), 'w') as f:
            f.write(f'yaml file://{local} {ros_distro}\n')

        env = dict(os.environ, ROSDEP_SOURCE_PATH=sources, ROS_HOME=os.path.join(tmp, 'ros_home'),
                   ROS_OS_OVERRIDE=f'{os_name}:{codename}:{codename}', DEB_BUILD_OPTIONS='nocheck')
        _run(['rosdep', 'update'], tmp, env)
        if install_built:
            _run(['apt-get', 'update'], tmp, env)   # build images ship without package lists

        for step in steps:
            work = os.path.join(tmp, 'build', step['package'])
            shutil.copytree(step['path'], work, ignore=shutil.ignore_patterns('build', 'install', 'log', '__pycache__', '.git'))
            _run(['git', 'init', '-q'], work, env)
            _run(['bloom-generate', 'rosdebian', '--ros-distro', ros_distro], work, env)

            with open(os.path.join(work, 'debian', 'changelog')) as f:
                changelog = f.read()
            with open(os.path.join(work, 'debian', 'changelog'), 'w') as f:
                f.write(stamp.stamp_changelog(changelog, version))
            with open(os.path.join(work, 'debian', 'control')) as f:
                control = f.read()
            with open(os.path.join(work, 'debian', 'control'), 'w') as f:
                f.write(stamp.pin_internal_depends(control, internal, version))

            if install_built:
                # Build-Depends straight from debian/control (the 'nocheck' profile drops the lint/test deps).
                _run(['apt-get', 'build-dep', '-y', '--no-install-recommends', '--allow-downgrades',
                      '--build-profiles=nocheck', './'], work, env)
            _run(['dpkg-buildpackage', '-us', '-uc', '-b', '-Pnocheck'], work, env)
            for deb in glob.glob(os.path.join(os.path.dirname(work), '*.deb')):
                dest = os.path.join(out_dir, os.path.basename(deb))
                shutil.move(deb, dest)
                built.append(dest)
                if install_built:
                    _run(['apt-get', 'install', '-y', '--allow-downgrades', dest], tmp, env)
    return built
