import os

from bloom_speech.envfile import DEFAULT_ENV_FILE, load_env, parse_env


def test_parse_env_handles_comments_blanks_and_quotes():
    values = parse_env([
        '# a comment', '', 'A=1', ' B = two ', 'C="quoted value"', "D='single'", 'E=a=b', 'NOEQUALS', '=novalue', 'F=',
    ])
    assert values == {'A': '1', 'B': 'two', 'C': 'quoted value', 'D': 'single', 'E': 'a=b', 'F': ''}


def test_load_env_adds_missing_but_never_overrides(tmp_path):
    f = tmp_path / 'robot.env'
    f.write_text('AZURE_TTS_KEY=from-file\nAZURE_TTS_REGION=eastus\n')
    env = {'AZURE_TTS_KEY': 'from-systemd'}
    assert load_env(env, [str(f)]) == [str(f)]
    assert env == {'AZURE_TTS_KEY': 'from-systemd', 'AZURE_TTS_REGION': 'eastus'}


def test_missing_or_unreadable_files_are_skipped(tmp_path):
    env = {}
    assert load_env(env, [str(tmp_path / 'absent.env'), str(tmp_path)]) == []  # a directory is not readable as a file
    assert env == {}


def test_earlier_files_win_over_later_ones(tmp_path):
    a, b = tmp_path / 'a.env', tmp_path / 'b.env'
    a.write_text('K=a\n')
    b.write_text('K=b\nL=b\n')
    env = {}
    load_env(env, [str(a), str(b)])
    assert env == {'K': 'a', 'L': 'b'}


def test_default_search_order_uses_bloom_env_file_then_etc(tmp_path):
    f = tmp_path / 'dev.env'
    f.write_text('X=1\n')
    env = {'BLOOM_ENV_FILE': str(f)}
    loaded = load_env(env)
    assert loaded[0] == str(f)
    assert env['X'] == '1'
    assert DEFAULT_ENV_FILE == '/etc/bloom/robot.env'

