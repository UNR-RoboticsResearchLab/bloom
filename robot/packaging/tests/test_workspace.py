import os

import pytest

from bloom_packaging import workspace


def make_pkg(root, name, deps=(), ignore=False, version='0.0.1'):
    d = root / name
    d.mkdir()
    body = ''.join(f'<depend>{x}</depend>' for x in deps)
    (d / 'package.xml').write_text(f'<package format="3"><name>{name}</name><version>{version}</version>{body}</package>')
    if ignore:
        (d / 'COLCON_IGNORE').write_text('')


def test_dependencies_build_first_and_external_deps_are_ignored(tmp_path):
    make_pkg(tmp_path, 'speech', ['engines', 'rclpy'])
    make_pkg(tmp_path, 'engines')
    make_pkg(tmp_path, 'node', ['msgs', 'rclcpp'])
    make_pkg(tmp_path, 'msgs')
    order = [p.name for p in workspace.build_order(workspace.find_packages(str(tmp_path)))]
    assert order.index('engines') < order.index('speech')
    assert order.index('msgs') < order.index('node')


def test_ignored_excluded_and_non_packages_are_skipped(tmp_path):
    make_pkg(tmp_path, 'a')
    make_pkg(tmp_path, 'b', ignore=True)
    make_pkg(tmp_path, 'c')
    (tmp_path / 'placeholder').mkdir()
    assert sorted(workspace.find_packages(str(tmp_path), exclude={'c'})) == ['a']


def test_cycle_is_an_error(tmp_path):
    make_pkg(tmp_path, 'a', ['b'])
    make_pkg(tmp_path, 'b', ['a'])
    with pytest.raises(ValueError, match='cycle'):
        workspace.build_order(workspace.find_packages(str(tmp_path)))
