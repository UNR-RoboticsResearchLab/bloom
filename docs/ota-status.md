# OTA system: status so far

As of 2026-10-29, branch `feature/fleet-remote-shell`. This page says what exists, what was checked,
and what is still missing. The design and contract are in [ota.md](ota.md); read that for how it works.

**Short version:** the robot-side agent and updater are built and unit-tested against stubs. Nothing
is wired end to end yet: the backend hub has no OTA methods, no `.deb` packages exist, and nothing
has run against real apt, dpkg or systemd.

## Built and tested

| Piece | Where | Notes |
|---|---|---|
| Manifest parsing and validation | `robot/agent/src/ota_manifest.*` | Ed25519 signature, expiry, suite allowlist, strict job-id and version patterns. |
| Updater script `fleet-ota` | `robot/agent/deploy/fleet-ota` | `stage` / `apply` / `cancel` / `status`; lock, `dpkg --configure -a`, rollback, atomic `state.json`. |
| systemd launch | `robot/agent/src/ota_system.*` | `systemd-run --unit=fleet-ota-<jobId> --collect --no-block`, arguments as an array, no shell. |
| Job state and persistent job record | `robot/agent/src/ota_state.*` | The record is the idempotency key, saved before launch. |
| Handlers, progress, reconnect catch-up | `robot/agent/src/ota_manager.*`, `tunnel.cpp` | `StartOta` / `CancelOta` / `GetOtaStatus`; `ReportOtaProgress` / `ReportInstalledVersion`. |
| Config, install, CI | `ota` block in `agent.json`, `install.sh`, `bloom-agent.service`, `robot-agent.yml` | Off by default. |
| Docs | [ota.md](ota.md) | Hub contract, manifest, signing, state machine, provisioning. |

**Verification run:** 123 C++ tests pass (all agent tests, including the OTA ones); `shellcheck` is clean
on `fleet-ota`; 23 bats tests run the real script against stubs for apt-get, dpkg, dpkg-query,
systemctl and the idle and health commands. I deliberately broke the script in two ways to confirm the
bats tests catch it (apt restriction, cancel handling). A documented `openssl` signing recipe is checked
against the agent's verifier with a fixed test vector.

## Not done

1. **Backend hub.** `IAgentClient` needs `StartOta` / `CancelOta` / `GetOtaStatus`; `RobotAgentHub` needs
   `ReportOtaProgress` / `ReportInstalledVersion`. Also missing: the signing key and manifest minting,
   per-robot job and version storage, an admin endpoint, and a dashboard control. Detail in
   [ota.md](ota.md#backend-work-still-to-do).
2. **Debian packages and the apt repo.** Nothing builds `.deb` files, the `bloom-robot-core` metapackage
   does not exist, and there is no repo. See below.
3. **Real-system testing.** No run against actual apt, dpkg or systemd, on a robot or in a container. Try a
   stage, an apply, and a deliberately failing health check on one robot before a fleet rollout.
4. **`robot-is-idle` and `robot-healthcheck`.** The updater calls them, but the executables are not
   written. Without them an apply fails (no idle check) or rolls back (no health check).
5. **Frontend.** The fleet card's version picker is filled from a release list (config plus GitHub
   Releases, see [fleet-dashboard.md](fleet-dashboard.md)), but its Update button stays disabled and
   there is no UI for watching an update.
6. **Updating the agent itself.** Out of scope for now; `install.sh`'s versioned directory plus `current`
   symlink is the natural basis.
7. **Bundle mode.** `sha256` is accepted in the manifest and ignored.

## Version numbering: findings

- Versions come from Nerdbank.GitVersioning (`version.json`): `1.0.<git height>`. It currently gives
  `1.0.20`. On `main` the string is plain; on any other branch it gets a `-g<commit>` suffix.
- **That suffix sorts the wrong way in Debian.** SemVer treats `1.0.20-gabc` as older than `1.0.20`;
  dpkg treats everything after the hyphen as the package revision, so it is *newer*
  (checked with `dpkg --compare-versions`). For `.deb` versions, turn the hyphen into a tilde:
  `1.0.20~gabc-1`, which sorts before `1.0.20-1` as intended.
- **ROS package versions are static.** `bloom-generate` takes the version from `package.xml`
  (`0.0.1`, `0.1.0`), giving e.g. `0.0.1-0noble`. If the metapackage pins those and the code changes
  without a bump, `apt install` has nothing to upgrade. The packaging step must overwrite every
  package's `debian/changelog` version with the mapped build version, and the metapackage must pin that
  exact string on all of them.
- The metapackage version is the one string OTA reports. It is a Debian version, not a SemVer, so the
  backend must not run it through the SemVer normaliser used for the agent's own version.
- Both `1.0.20-1` and `1.0.20~gabc-1` pass the agent's manifest version pattern.

## `.deb` packaging: where it stands

Feasible, with `bloom-generate rosdebian` (which works offline here) plus a hand-built metapackage.
Checked, not built: `debhelper` is not installed here, so no real `dpkg-buildpackage` run happened.

| Item | Status |
|---|---|
| Speech engines were not part of any ROS package | **Fixed**: now `bloom_speech_engines` (colcon build checked, `bloom-generate` produces a package). |
| Secrets baked into packages (`.env` in `bloom_speech`'s install) | **Fixed**: nodes read the environment (systemd `EnvironmentFile=`, or `BLOOM_ENV_FILE`); no package ships the file; the metapackage's postinst creates `/etc/bloom/robot.env` empty and root-only if missing and never overwrites it. |
| Version mapping, stamping, build order, metapackage | **Built and unit-tested**: `robot/packaging/` (31 tests, includes a real `dpkg-deb` metapackage and dpkg ordering checks). |
| `bloom_node` cannot resolve `bloom_msgs` when generated alone | **Solved** with a temporary local rosdep source (`ROSDEP_SOURCE_PATH`); verified that `bloom-generate` then resolves it. |
| A real `--execute` build | **Never run**: needs `debhelper`, root and a native arm64 runner. A manual-only CI job exists (`robot-packages.yml`). |
| Azure and OpenAI SDKs are pip-only | **Open decision**: vendor a directory of packages (`.pth`, no numpy) or build `.deb`s for each SDK. |
| arm64 build | CI job targets `ubuntu-24.04-arm`; M-SARG's hardware is a Raspberry Pi 5 (arm64). |
| `blsm_unr` submodule | Open: not referenced by any `package.xml` or build file; confirm whether it belongs in the image. |
| `bloom_interface` | Empty placeholder; leave out. |
| `lintian` | Not installed; add to the packaging CI step. |

## Decisions that are yours

1. How to ship the pip-only SDKs (above). This blocks a working `bloom-robot-core`.
2. Whether the agent should be allowed `sudo systemd-run` only, rather than all of sudo, via a templated
   unit (see [ota.md](ota.md#why-systemd-run-not-a-child-process-not-a-template-unit)).
3. Who holds the manifest signing key and how it is rotated.
4. Whether old package versions stay in the apt repo indefinitely (rollback reinstalls the previous
   version from the repo).

## Suggested order from here

1. ~~Packaging script (version mapping, stamping, metapackage)~~ done. Next: run the first real
   `--execute` build on an arm64 runner, and decide the SDK packaging approach.
2. Stand up a test apt repo and run `fleet-ota` against a container with real apt and systemd.
3. Add the backend hub methods and a minimal admin trigger; test on one robot.
4. Write `robot-is-idle` and `robot-healthcheck`; add the dashboard control.
