import shutil
import subprocess

import pytest

from bloom_packaging.version import semver_to_debian

needs_dpkg = pytest.mark.skipif(not shutil.which('dpkg'), reason='dpkg not available')


def test_release_and_prerelease_mapping():
    assert semver_to_debian('1.0.20') == '1.0.20-1'
    assert semver_to_debian('1.0.20-g4fc39fd5a3') == '1.0.20~g4fc39fd5a3-1'
    assert semver_to_debian('1.2.3-rc-1.2') == '1.2.3~rc.1.2-1'     # inner hyphen can't be confused with the revision
    assert semver_to_debian('1.2.3+build.5') == '1.2.3+build.5-1'
    assert semver_to_debian('1.0.20', revision='2') == '1.0.20-2'


@pytest.mark.parametrize('bad', ['1.0', 'v1.0.0', '1.0.0-', '', '1.0.0 beta', '1.0.0+'])
def test_rejects_non_semver(bad):
    with pytest.raises(ValueError):
        semver_to_debian(bad)


def test_rejects_unsafe_revision():
    with pytest.raises(ValueError):
        semver_to_debian('1.0.0', revision='1; rm')


def dpkg_lt(a, b):
    return subprocess.run(['dpkg', '--compare-versions', a, 'lt', b]).returncode == 0


@needs_dpkg
def test_nbgv_branch_builds_sort_before_the_release_they_precede():
    # The reason this module exists: raw SemVer would get this the wrong way round under dpkg.
    assert not dpkg_lt('1.0.20-g4fc39fd', '1.0.20')
    pre, rel = semver_to_debian('1.0.20-g4fc39fd'), semver_to_debian('1.0.20')
    nxt, nxt_pre = semver_to_debian('1.0.21'), semver_to_debian('1.0.21-gabc')
    assert dpkg_lt(pre, rel) and dpkg_lt(rel, nxt_pre) and dpkg_lt(nxt_pre, nxt)
    assert dpkg_lt(semver_to_debian('1.0.9'), semver_to_debian('1.0.10'))


@needs_dpkg
@pytest.mark.parametrize('v', ['1.0.20', '1.0.20-g4fc39fd5a3', '2.1.0-rc-1', '1.0.0+meta'])
def test_results_are_valid_debian_versions(v):
    assert subprocess.run(['dpkg', '--validate-version', semver_to_debian(v)]).returncode == 0
