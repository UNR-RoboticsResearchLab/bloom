# On-demand OTA updates for the robot ROS stack

Operator-triggered software updates, delivered as Debian packages from a private, GPG-signed apt
repository. **Nothing polls and nothing updates by itself.** An update happens only when an admin
sends a signed manifest through the fleet dashboard.

| Piece | Where | Job |
|---|---|---|
| `bloom-robot-core` metapackage | apt repo | Pins exact versions of every ROS package, so one version string describes the robot's whole software state. |
| `robot-agent` (C++) | `robot/agent/src/ota_*.cpp` | The **trigger and reporter**: validates the manifest, launches the updater, forwards progress. Does no package work itself. |
| `fleet-ota` (bash) | `robot/agent/deploy/fleet-ota` | The updater. Runs as root in its own systemd unit. Drives apt, dpkg, the ROS service, the health check and rollback. |
| Backend hubs | `backend/Services/RemoteShell/` | Control plane only. **Not yet implemented for OTA**: see [Backend work](#backend-work-still-to-do). |

SignalR carries only small JSON: the manifest down, progress events up. The robot downloads packages
itself over HTTPS from the apt repo, and **apt verifies the repo signature**. `--allow-unauthenticated`
is never used.

## Why `systemd-run`, not a child process, not a template unit

`fleet-ota` is started with `systemd-run --unit=fleet-ota-<jobId> --collect --no-block`. It therefore
lives in its own transient unit and cgroup. `bloom-agent.service` uses `KillMode=control-group`, so
anything the agent forks as a child would be killed with it when the agent restarts or crashes,
possibly in the middle of a dpkg transaction. In its own unit, dpkg is unaffected by the agent.

I chose `systemd-run` over a templated oneshot (`fleet-ota@.service`) because the unit is created on
demand with a name derived from the validated job id, per-job settings travel as `--setenv`, and
there is no extra unit file to install and keep in step with the script. The cost is that the agent
needs root to call `systemd-run`: it uses `sudo -n`, which the `bloom-ops` user already has
(`install.sh` grants it passwordless sudo). A template unit would let you narrow the sudoers rule to
`systemctl start fleet-ota@*`; if you want that, it is a contained change in `ota_system.cpp`.

## Hub contract

Extends the remote-shell contract (`robot/agent/src/protocol.h`, `RobotAgentHub.cs`,
`ShellHubClients.cs`). Same hub, `/hubs/robot-agent`; same JSON hub protocol.

Server → robot (`IAgentClient`):

| Method | Arguments | Behaviour |
|---|---|---|
| `StartOta` | `manifest` (JSON object; a JSON string holding one is also accepted) | Validate, then launch. Rejections are reported as `rejected`. |
| `CancelOta` | `jobId` | See [Cancel](#cancel). |
| `GetOtaStatus` | none | Robot answers with `ReportInstalledVersion` and `ReportOtaProgress` for its newest job. |

Robot → server (`RobotAgentHub`):

| Method | Arguments | When |
|---|---|---|
| `ReportOtaProgress` | `jobId: string, state: string, percent: int (0-100), detail: string` | On every change of state, percent or detail; and re-sent on reconnect. `detail` is at most 300 characters of plain text. |
| `ReportInstalledVersion` | `version: string` | On **every connect and reconnect**, and after an apply finishes (`success` or `rolled_back`). Empty string = metapackage not installed. |

All of these are fire-and-forget (no completion is awaited), so the server must treat
`ReportOtaProgress` as **idempotent**: it is deliberately sent more than once (see
[Reconnects](#reconnects-and-restarts)). Key on `(jobId, state, percent, detail)`, or simply keep the latest.

If OTA is disabled on a robot (`ota.enabled` false, the default), `StartOta` is logged and ignored,
and the server will get no answer. Gate on the agent version/capability on the backend side.

## Manifest

```json
{
  "jobId": "9b2f6c1e-example-0001",
  "action": "apply",
  "version": "1.4.2-1jazzy",
  "source": { "type": "apt", "suite": "jazzy-stable" },
  "sha256": null,
  "signature": "r/MpwXjJJHlQx+DRZwGhqct144KqtDAY6VGNyqVk17fAO0lU/gZSXj1q9HFEsZEowsHB/VECAWRpNmW10YjaAA==",
  "expiresAt": "2999-01-01T00:00:00Z"
}
```

This example is genuinely signed, with a throwaway test key whose public half is in
`robot/agent/tests/ota_manifest_test.cpp`. It is valid on a robot configured with that key and the
`jazzy-stable` suite, and nowhere else.

| Field | Rule |
|---|---|
| `jobId` | Required. 1-64 chars of `[A-Za-z0-9_-]`. It becomes part of a systemd unit name and file names. Use a new id for every attempt, including retries. |
| `action` | `"stage"` or `"apply"`. |
| `version` | Required. A Debian version without an epoch: starts with a digit, then `[A-Za-z0-9.+~-]`, at most 64 chars. |
| `source.type` | Must be `"apt"`. |
| `source.suite` | Must be in the robot's `ota.allowed_suites`. Policy check only: the robot's own sources file decides where packages come from. |
| `sha256` | Optional, 64 hex chars. Accepted, signed and ignored: reserved for a future bundle mode. |
| `signature` | Base64 Ed25519 signature, below. |
| `expiresAt` | UTC, exactly `YYYY-MM-DDTHH:MM:SSZ`. Rejected once the robot's clock reaches it. |

### Signing

The signature covers this exact text (UTF-8, `\n` separators, no trailing newline). `sha256` is the
empty string when absent, so that line is empty:

```
bloom-ota-v1
<jobId>
<action>
<source.type>
<source.suite>
<version>
<sha256>
<expiresAt>
```

There is no JSON canonicalisation to get wrong. A signed `stage` cannot be replayed as an `apply`
because `action` is signed. With the OpenSSL CLI:

```bash
openssl genpkey -algorithm ed25519 -out ota-signing.pem          # keep this private
openssl pkey -in ota-signing.pem -pubout -out ota-signing.pub    # this goes on the robots
printf 'bloom-ota-v1\n%s\n%s\napt\n%s\n%s\n\n%s' "$JOB" "$ACTION" "$SUITE" "$VERSION" "$EXPIRES" > payload
openssl pkeyutl -sign -inkey ota-signing.pem -rawin -in payload | base64 -w0
```

The agent verifies with OpenSSL (`ota_manifest.cpp`). The key file is read fresh for each manifest,
so rotating the key is just replacing the file.

### Validation order

On any failure the job is reported `rejected` with a reason, **recorded** (so a replay is ignored)
and never launched: source type, suite allowlist, expiry, then the signature. Malformed fields
(`jobId`, `action`, `version`...) are rejected before those. A manifest without a usable `jobId` is only
logged, since there is no job to report against.

Nothing from a manifest is ever put in a shell command line. The agent passes arguments as an array
to `execvp`; the only manifest strings that reach `fleet-ota` are the job id (`--setenv` and the unit
name) and the version (an argument), and the script re-validates both with the same patterns.

## State machine

```
            ┌──────────────────────────── cancelled (before install begins)
            │
 queued ──► staging ──► staged                                            (action: stage)

 queued ──► applying ──► healthcheck ──► success                          (action: apply)
    │          │              │
    │          └──────────────┴────► rolled_back   (previous version restored)
    │                                failed        (could not install, or could not roll back)
    └──► deferred (robot not idle)
 rejected: manifest refused by the agent, never launched
```

`queued` is reported by the agent as soon as a manifest is accepted; everything after comes from
`fleet-ota`'s state file. Terminal states: `staged`, `success`, `rolled_back`, `failed`, `deferred`,
`rejected`, `cancelled`. `percent` is a coarse step marker, not a download meter.

### Two phases

1. **stage** — `dpkg --configure -a`, `apt-get update` against *only* the fleet sources file,
   `apt-get install --download-only bloom-robot-core=<version>`. The robot keeps running normally and is
   unchanged. Reports `staged`.
2. **apply** — `dpkg --configure -a`; check `robot-is-idle` (not idle → `deferred`, nothing touched);
   record the installed version; install `bloom-robot-core=<version>` with `--allow-downgrades`; restart
   the ROS service; run the health check; report `success`. If the version is already installed it
   reports `success` without doing anything. Stage is optional but makes apply faster and means a flaky
   network cannot interrupt the install.

### Failure and rollback

| What fails during `apply` | Result |
|---|---|
| Robot not idle | `deferred`. Nothing changed. Send a new manifest later. |
| `dpkg --configure -a` fails | `failed`. Package database needs a human. |
| Install of the target fails | Reinstall previous version, restart service, `rolled_back`. |
| ROS service fails to restart | Same. |
| Health check fails or times out | Same. |
| Rollback itself fails | `failed`. The robot is in an unknown state; the detail says rollback failed. |
| No previous version recorded (first install) | `failed`: there is nothing to roll back to. |
| `fleet-ota` killed or crashes | `fleet-ota` writes `failed` from its exit trap if it can; if it is killed outright the agent notices the unit is gone without a terminal state (after a 20 s grace) and reports `failed`. The next run's `dpkg --configure -a` repairs an interrupted install. |
| Host reboots mid-update | As above, on the agent's next connection. |

Rollback reinstalls the previous version **from the apt repo**, so the repo must keep old versions of
`bloom-robot-core` and everything it pins for as long as any robot might need to return to them.
Stage failures (`apt-get update` or download) never touch the installed system.

The health check (`ota.healthcheck_command`) runs once, after the restart, under `timeout
<healthcheck_timeout_seconds>`. It should wait for the stack to become ready itself, and exit 0 for
healthy.

### Cancel

`CancelOta` is honoured only **before install begins**: while a job is `queued`, or a `stage` job is
`staging`. The agent asks `fleet-ota cancel <jobId>` to drop a flag file; the updater checks it between
steps, so a running `apt-get update`/download finishes its step first, then the job ends `cancelled`.
For an `apply` that has reached `applying` or `healthcheck`, the cancel is refused and the agent
re-sends the job's state with a detail saying so. Cancel never signals dpkg. Unknown or finished jobs
are ignored.

## Idempotency, reconnects and restarts

* **Every `jobId` the agent has seen (accepted or rejected) is recorded** in
  `ota.job_store_path` (`/var/lib/bloom-agent/ota-jobs.json`, written atomically *before* launching).
  A duplicate `StartOta` after a reconnect or an agent restart is ignored. The newest 50 jobs are
  kept; a job still running is never dropped.
* One update at a time: a second manifest while one is running is `rejected`, and `fleet-ota` also
  holds an exclusive `flock`.
* Progress is read from `/var/lib/fleet-ota/state.json` about once a second while a job is running and
  connected (a poll, not inotify, which is plenty at this rate and keeps the single-threaded loop simple).
* **On every (re)connect** the agent sends `ReportInstalledVersion` (from `dpkg-query`), catches up
  with the state file, then re-sends every job whose latest state was never handed to a connection,
  which is how a job that finished while the robot was offline gets reported. It also re-sends the
  newest job unconditionally, in case the previous connection died with the report undelivered.
* Updates run in their own unit, so a dropped tunnel, an agent restart or a crash never affects an
  install in progress.
* OTA code is isolated from the tunnel: every handler catches its exceptions, nothing blocks for more
  than a few seconds, and the remote shell and key handling are untouched.

## Provisioning a robot

OTA is **off by default**. Everything below is a one-time step per robot (or in the image).

1. **Install the agent release** (`install.sh`). It installs `fleet-ota` next to the agent and creates
   `/var/lib/fleet-ota`; the service creates `/var/lib/bloom-agent` (`StateDirectory=`).
2. **apt source**, single-line format in its own file (`/etc/apt/sources.list.d/bloom-fleet.list`):
   ```
   deb [arch=arm64 signed-by=/usr/share/keyrings/bloom-fleet.gpg] https://apt.example.edu/bloom jazzy-stable main
   ```
   It must be the one-line `.list` format: `fleet-ota` selects it with `Dir::Etc::sourcelist`, which does
   not read deb822 `.sources` files. `fleet-ota` uses *only* this file (`Dir::Etc::sourceparts=-`), so
   Ubuntu's own sources are never consulted by an update.
3. **Repo signing key** (the apt repo's GPG key, unrelated to the manifest key):
   `gpg --dearmor < bloom-fleet.asc > /usr/share/keyrings/bloom-fleet.gpg`. This is what makes apt verify
   packages.
4. **Pin**, so ordinary `apt upgrade` can't pull these packages from elsewhere
   (`/etc/apt/preferences.d/bloom-fleet`):
   ```
   Package: bloom-* ros-jazzy-bloom-*
   Pin: origin apt.example.edu
   Pin-Priority: 990
   ```
5. **Manifest signing public key** at `/etc/bloom-agent/ota-signing.pub` (Ed25519, PEM), mode 0644.
   Only the public key is ever on a robot.
6. **Two executables you supply**, both exit status only, run as root by `fleet-ota`:
   `/usr/local/bin/robot-is-idle` (0 = idle, safe to restart the stack) and
   `/usr/local/bin/robot-healthcheck` (0 = healthy). A missing or non-executable idle check **fails** an
   apply rather than skipping the check; a missing health check counts as unhealthy and rolls back.
7. **Enable it** in `/etc/bloom-agent/agent.json`, then `systemctl restart bloom-agent`:
   ```json
   "ota": { "enabled": true, "allowed_suites": ["jazzy-stable"] }
   ```
8. **systemd**: nothing to install for the updater unit (`systemd-run` creates it). The agent user needs
   passwordless sudo for `systemd-run` and `fleet-ota`, which `bloom-ops` has.

Check on a robot: `journalctl -u bloom-agent | grep 'ota job='` for the agent, and
`journalctl -u 'fleet-ota-*'` for the updater (transient units are collected on exit but their journal
entries remain).

### Agent settings (`ota` object in `agent.json`)

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `false` | Master switch. |
| `allowed_suites` | `[]` | Manifest `source.suite` must be one of these. Empty rejects everything. |
| `metapackage` | `bloom-robot-core` | The package that pins the whole stack. |
| `apt_sources_path` | `/etc/apt/sources.list.d/bloom-fleet.list` | The only apt source the updater uses. |
| `signing_key_path` | `/etc/bloom-agent/ota-signing.pub` | Manifest verification key. |
| `healthcheck_timeout_seconds` | `120` | Upper bound for the health check (1-3600). |
| `idle_command` | `/usr/local/bin/robot-is-idle` | Exit 0 = idle. |
| `healthcheck_command` | `/usr/local/bin/robot-healthcheck` | Exit 0 = healthy. |
| `ros_service` | `bloom.service` | Restarted after install. |
| `updater_path` | `/opt/bloom-agent/current/fleet-ota` | The script. |
| `state_dir` | `/var/lib/fleet-ota` | Where `state.json` lives. |
| `job_store_path` | `/var/lib/bloom-agent/ota-jobs.json` | The idempotency record. |

`fleet-ota` receives these as `FLEET_OTA_*` environment variables from the agent (never from a
manifest), so there is one source of truth.

### Logging

The agent logs one `key=value` line per event to the journal, matching the rest of the agent
(syslog-prefixed stderr): `ota job=<id> state=failed percent=40 detail="..."`, plus `accepted`,
`rejected: <reason>`, `duplicate StartOta ignored`, `CancelOta refused ...`. `fleet-ota` prints the
same shape (`fleet-ota job=<id> state=...`) into its own unit's journal.

## Testing

* C++: `ctest --test-dir robot/agent/build` (manifest validation, signature, job-id idempotency including
  across restarts, state transitions, cancel rules, reconnect re-send, launch argument construction).
* Updater: `shellcheck -x robot/agent/deploy/fleet-ota` and `bats robot/agent/tests/fleet-ota`. The bats
  tests run the real script against stubs for apt-get, dpkg, dpkg-query, systemctl and the idle and
  health commands (`tests/fleet-ota/stub`), so they never touch the machine's packages or services.
  CI runs both (`.github/workflows/robot-agent.yml`).
* Not covered by automated tests: real apt/dpkg/systemd behaviour. Try a stage and an apply on one
  robot (or a container with systemd) before a fleet rollout, including a deliberately failing health check.

## Backend work still to do

The hub is in this repo but OTA is not wired into it. The backend needs to:

1. Add to `IAgentClient` (`ShellHubClients.cs`): `StartOta(object manifest)`, `CancelOta(string jobId)`,
   `GetOtaStatus()`.
2. Add to `RobotAgentHub`: `ReportOtaProgress(string jobId, string state, int percent, string detail)` and
   `ReportInstalledVersion(string version)`. The robot id comes from the JWT as for the other methods.
   Handle repeated reports idempotently.
3. Hold an Ed25519 signing key (outside the repo), build the signing text above, mint manifests with a
   fresh `jobId` and a short `expiresAt` (minutes), and store jobs and the installed version per robot
   (a migration, an admin-only endpoint and a dashboard control).
4. Mirror the contract in `RobotTerminal.jsx`/the fleet view as needed, and decide on the version
   semantics of `ReportInstalledVersion`: it is the Debian version of `bloom-robot-core`, **not** a
   SemVer like the agent's, so don't pass it through the SemVer normaliser.
5. Gate `StartOta` on robots that have OTA enabled; an agent with it off silently ignores the call.

## Out of scope / follow-ups

* No payload transfer over SignalR, no polling, no auto-update, no OS or ROS distro upgrades.
* **Updating the agent itself** is a follow-up. `install.sh` already uses a versioned directory and a
  `current` symlink, which is the natural basis for it. Note `updater_path` points through that
  symlink, so the updater a job runs is whatever `current` points to when it launches.
* `sha256`/bundle mode is reserved in the manifest but not implemented.
* `source.suite` is an allowlist check only; it is not passed to apt (`-t`). The repo's sources file and
  the exact `name=version` pin determine what is installed.
