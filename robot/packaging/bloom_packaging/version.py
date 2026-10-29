"""SemVer (what Nerdbank.GitVersioning emits) -> Debian version.

The trap: nbgv gives `1.0.20-g4fc39fd` off the main branch. SemVer sorts that *before* `1.0.20`, but
dpkg reads everything after the last hyphen as the package revision, so it sorts *after*. Mapping the
prerelease separator to `~` restores the intended order:

    1.0.20~g4fc39fd-1  <  1.0.20-1  <  1.0.21~gabc-1  <  1.0.21-1
"""
import re

_SEMVER = re.compile(
    r'^(?P<core>\d+\.\d+\.\d+)'
    r'(?:-(?P<pre>[0-9A-Za-z.-]+))?'
    r'(?:\+(?P<meta>[0-9A-Za-z.-]+))?$')
_REVISION = re.compile(r'^[0-9A-Za-z.+~]+$')


def semver_to_debian(semver: str, revision: str = '1') -> str:
    m = _SEMVER.match(semver.strip())
    if not m:
        raise ValueError(f'not a SemVer 2.0 version: {semver!r}')
    if not _REVISION.match(revision):
        raise ValueError(f'invalid Debian revision: {revision!r}')
    version = m.group('core')
    if m.group('pre'):
        # '-' inside a prerelease would be ambiguous with the revision separator.
        version += '~' + m.group('pre').replace('-', '.')
    if m.group('meta'):
        version += '+' + m.group('meta').replace('-', '.')
    return f'{version}-{revision}'
