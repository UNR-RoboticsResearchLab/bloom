# Debian packaging for the robot software

Turns the ROS packages under `robot/src/` into `.deb`s that all carry one build version, plus a
metapackage that pins them. This is what [OTA updates](../../docs/ota.md) install: the metapackage
version is the robot's whole software state. Used by this repo's CI and by m-sarg's.

```bash
cd robot/packaging
python3 -m bloom_packaging version "$(nbgv get-version -v SemVer2)"       # 1.0.20~g4fc39fd5a3-1
python3 -m bloom_packaging build --src ../src --out dist \
        --semver "$(nbgv get-version -v SemVer2)"                          # prints the plan
python3 -m bloom_packaging build ... --execute --install-built             # really builds (CI container only)
python3 -m bloom_packaging meta --name bloom-robot-core --semver ... --from-dir dist --out dist \
        --extra-depends libportaudio2 --env-file /etc/bloom/robot.env
env -u PYTHONPATH python3 -m pytest packaging/tests                        # tests
```

## What it does, and why

- **Version mapping** (`version.py`). nbgv emits `1.0.20-g4fc39fd` off `main`. SemVer sorts that
  *before* `1.0.20`; dpkg treats it as a revision and sorts it *after*. It becomes `1.0.20~g4fc39fd-1`.
- **Stamping** (`stamp.py`). `bloom-generate` takes versions from `package.xml` (`0.0.1`, never
  changing), so a code change would ship under the same version and `apt` would see nothing to
  upgrade. Every package's changelog is rewritten to the build version, and references between our
  own packages become exact `(= version)` pins.
- **Build order** (`workspace.py`). Dependencies first. Each finished package is installed
  (`--install-built`) so the next builds against it; rosdep resolves our own package names through
  a temporary local source (`ROSDEP_SOURCE_PATH`), never `/etc/ros`.
- **Metapackage** (`metapackage.py`). `Depends: pkg (= ver), ...` for every `.deb` in the directory.

## Secrets are not packaged

`--env-file /etc/bloom/robot.env` makes the metapackage's `postinst` create that file **empty and
root-only (0600) if it is missing, and never touch it otherwise**. The file is not in the package's
payload, so upgrades don't overwrite it and rollbacks don't restore an old copy. systemd loads it with
`EnvironmentFile=`; see `bloom_speech/envfile.py`.

## Not done / not verified

- A real `--execute` build has never been run: it needs `debhelper`, root and a native arm64 runner.
  Only the planning, stamping, ordering and metapackage steps are tested (`tests/`).
- Pip-only dependencies (Azure/OpenAI SDKs, sounddevice, ...) are not packaged yet (planned: a
  `bloom-robot-pydeps` package).
- No apt repo or signing yet; `dist/` is just a directory.
