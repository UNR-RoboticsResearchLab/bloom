import os
import shutil
import subprocess

import pytest

from bloom_packaging import metapackage as meta

pytestmark = pytest.mark.skipif(not shutil.which('dpkg-deb'), reason='dpkg-deb not available')


def spec(**kw):
    base = dict(name='bloom-robot-core', version='1.0.20~gabc-1', description='test',
                pins={'ros-jazzy-bloom-node': '1.0.20~gabc-1', 'ros-jazzy-bloom-msgs': '1.0.20~gabc-1'},
                extra_depends=['libportaudio2'])
    base.update(kw)
    return meta.MetaSpec(**base)


def test_control_pins_every_package_exactly():
    text = meta.control_text(spec())
    assert 'Depends: ros-jazzy-bloom-msgs (= 1.0.20~gabc-1), ros-jazzy-bloom-node (= 1.0.20~gabc-1), libportaudio2' in text
    assert 'Version: 1.0.20~gabc-1' in text and 'Architecture: all' in text


def test_a_real_deb_is_built_with_the_right_metadata(tmp_path):
    deb = meta.build(spec(), str(tmp_path))
    assert meta.read_deb_field(deb, 'Package') == 'bloom-robot-core'
    assert meta.read_deb_field(deb, 'Version') == '1.0.20~gabc-1'
    assert '(= 1.0.20~gabc-1)' in meta.read_deb_field(deb, 'Depends')


def test_no_env_file_is_part_of_the_payload(tmp_path):
    deb = meta.build(spec(env_files=['/etc/bloom/robot.env']), str(tmp_path))
    listing = subprocess.run(['dpkg-deb', '-c', deb], capture_output=True, text=True, check=True).stdout
    assert 'robot.env' not in listing          # dpkg must never own, overwrite or restore the secrets
    assert 'postinst' in subprocess.run(['dpkg-deb', '-I', deb], capture_output=True, text=True, check=True).stdout


def run_postinst(tmp_path, files):
    script = tmp_path / 'postinst'
    script.write_text(meta.postinst_text(files))
    root = tmp_path / 'root'
    subprocess.run(['sh', str(script)], check=True, env=dict(os.environ, DPKG_ROOT=str(root)))
    return root


def test_postinst_creates_an_empty_private_file_when_missing(tmp_path):
    root = run_postinst(tmp_path, ['/etc/bloom/robot.env'])
    f = root / 'etc/bloom/robot.env'
    assert f.exists() and f.read_text() == ''
    assert (f.stat().st_mode & 0o777) == 0o600


def test_postinst_never_touches_an_existing_file(tmp_path):
    f = tmp_path / 'root/etc/bloom/robot.env'
    f.parent.mkdir(parents=True)
    f.write_text('AZURE_TTS_KEY=secret\n')
    f.chmod(0o640)
    run_postinst(tmp_path, ['/etc/bloom/robot.env'])
    assert f.read_text() == 'AZURE_TTS_KEY=secret\n'
    assert (f.stat().st_mode & 0o777) == 0o640


@pytest.mark.parametrize('bad', ['relative/path', '/etc/x y', '/etc/$(id)', "/etc/a'b", '/etc/a;b'])
def test_unsafe_env_paths_are_refused(bad):
    with pytest.raises(ValueError):
        meta.postinst_text([bad])
