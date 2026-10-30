# Fleet dashboard: adding robots, flashing M-SARG cards, and the version list

The admin dashboard's **Fleet** section is where robots are added, given credentials, reached over
the remote shell, and (once the backend side lands) updated. This page covers three parts of it:

1. [Adding a robot and flashing an M-SARG SD card](#adding-a-robot-and-flashing-an-m-sarg-card)
2. [The release list behind the "Target version" picker](#the-release-list-target-version-picker)
3. [Labelling a release](#labelling-a-release) (git tag, GitHub Release, channel)

For the remote shell see the README's "Fleet remote shell"; for the update protocol see
[ota.md](ota.md) and [ota-status.md](ota-status.md).

## Adding a robot and flashing an M-SARG card

Fleet → **Add robot** asks for a name (unique) and a model (`Blossom` or `M-SARG` are suggested; it
is free text). The backend creates the robot and returns its id and a **secret that is shown once**.
The next screen can't be closed until you tick "I've saved the secret". It offers:

- **Download provisioning file**: `<name>.provision.json` (`version`, `backend_url`, `robot_id`,
  `name`, `secret`). It holds the secret in plain text; delete it once used.
- **Commands to run on the robot**: `install.sh --provision-file` for a first install of the
  agent, `provision.sh` for a robot that already has it.
- **Flash an M-SARG SD card**: a form that writes the `deploy/deploy_sd.sh` command from the
  [m-sarg](https://github.com/UNR-RoboticsResearchLab/m-sarg) repo. It opens by default when the
  model is M-SARG, and is available for any robot. The same screen appears under Fleet →
  **Credentials** when you issue new credentials.

### The flash form

The browser can't write a card or run a root script, so the form only builds the command. Copy it and
run it from your `m-sarg` checkout on a Linux machine with a card reader. The script builds the whole
image offline (20+ minutes the first time) and bakes in this robot's id, secret and backend URL
(`/etc/m-sarg/bloom.env`, root-only), so the robot connects on first boot with nothing set up on it.

| Field | Becomes | Notes |
|---|---|---|
| Robot number (1-100) | `--robot-id N` | Hostname `m-robot-NN` and `ROS_DOMAIN_ID`. Filled in from a trailing number in the robot's name. Use a different number per card. |
| Write to | `--device /dev/sdX` or `--image PATH` | The device is erased; the script asks you to retype it. |
| Wi-Fi networks | `--wifi-ssid/--wifi-psk`, `--wifi-eap-ssid/--wifi-eap-identity/--wifi-eap-password`, `--wifi-open-ssid` | One or more, in roaming order. `"` and `\` are rejected, as the script does. |
| SSH public key | `--ssh-pubkey` | The image allows key logins only. |
| Azure speech `.env` | `--env-file` | Required with the bloom service. |
| Provisioning file | `--bloom-provision-file` | Defaults to `~/Downloads/<name>.provision.json`. |
| `gh auth token` | `--repo-token "$(gh auth token)"` | On by default; the m-sarg repo is private. |
| Master image (optional) | `--base-image-url file://...` | Faster repeat flashes (m-sarg's README, "master image"). |

`--enable-bloom-service` is always included. Until the form is complete it lists what is missing
instead of showing a command. Nothing typed in it is sent to the backend; Wi-Fi passwords appear only in
the command text (and so in your shell history). **Each card needs its own robot**: add a new one
here for every card, because the secret is per robot and the remote shell session is tied to it.
For many cards use m-sarg's `deploy/flash_batch.sh` with one provisioning file per robot.

Code: `frontend/src/components/dashboard/MsargFlashBuilder.jsx` (form),
`frontend/src/utils/msargDeploy.js` (command builder and the validation that mirrors
`deploy/lib/args.sh`), shown from `SecretReveal.jsx`.

## The release list ("Target version" picker)

Each fleet card has a release channel switch (Prod / Canary / Dev) and a **Target version** dropdown.
The versions come from `GET /api/robot/releases` (admin only), which the dashboard loads once per
visit:

```json
{ "channels": { "prod": ["1.0.21-1"], "canary": ["1.0.22~g4fc39fd-1"], "dev": [] },
  "warnings": [] }
```

Each list is newest first, ordered the way the robot's `dpkg` orders versions (so `1.0.20~gabc-1`
comes before `1.0.20-1`). The values are Debian versions of the `bloom-robot-core` metapackage, which
is what [OTA](ota.md) pushes. The backend merges two sources:

1. **Config**: versions you list by hand under `Releases:Channels`.
2. **GitHub Releases** of one repository, read by the backend (so a token never reaches the browser,
   and one cached call serves every admin).

If GitHub can't be read, the configured versions are still returned and `warnings` says so; the
dashboard shows it as an amber note.

### Configuration

`backend/appsettings.json`, section `Releases`; any key can be set by environment variable
(`Releases__GitHub__Token`).

| Key | Default | Meaning |
|---|---|---|
| `Channels:prod` / `canary` / `dev` | `[]` | Debian versions to always list in that channel. Invalid ones are logged and ignored. |
| `GitHub:Repository` | `UNR-RoboticsResearchLab/bloom` | `owner/repo`; empty turns GitHub off. |
| `GitHub:Token` | empty | Optional. Needed for a private repo; raises the anonymous limit of 60 requests an hour. |
| `GitHub:TagPrefix` | `robot-v` | Only tags starting with this are robot releases (other tags, like the demo tags, are skipped). Empty takes every tag. |
| `GitHub:DevPattern` | `(^\|[-.~+])dev` | A GitHub pre-release whose SemVer matches goes to `dev` instead of `canary`. |
| `GitHub:CacheMinutes` | `10` | How long a good GitHub answer is reused. A failed one is retried after 1 minute. |

How a GitHub Release maps to a channel:

| Release | Channel |
|---|---|
| Normal (not pre-release) | `prod` |
| Pre-release | `canary`, or `dev` if its SemVer matches `DevPattern` |
| Draft, wrong prefix, or a tag that isn't SemVer | not listed |

The **Update** button is still disabled: nothing on the backend sends an update to a robot yet (see
[ota-status.md](ota-status.md)). The "(current)" marker compares against the agent's own version, so
it won't match a package version until robots report their installed `bloom-robot-core`.

Code: `backend/Services/ReleaseCatalog/` (`ReleaseCatalogService`, `DebianVersion`,
`ReleaseCatalogOptions`), the endpoint in `RobotController`, and `RobotFleetCard.jsx` /
`AdminDashboard.js` on the frontend. Tests: `tests/Services/ReleaseCatalogServiceTests.cs`.

## Labelling a release

A robot release is a git tag `robot-v<SemVer>` plus a GitHub Release. Make one with the **Robot
release** workflow ([robot-release.yml](../.github/workflows/robot-release.yml)): Actions → Robot
release → Run workflow → choose `canary` or `prod`. (It is manual; it never runs on a push.)

| Tag | GitHub Release | Channel | Package version on the robot |
|---|---|---|---|
| `robot-v1.0.21` | normal | prod | `1.0.21-1` |
| `robot-v1.0.21-g4fc39fd` | pre-release | canary | `1.0.21~g4fc39fd-1` |

- The tag holds the SemVer that Nerdbank.GitVersioning gives that commit. The Debian form is derived
  from it (same rule as `robot/packaging/bloom_packaging/version.py`) because git forbids `~` in tag
  names.
- `prod` only from `main` (where the version has no suffix); `canary` from any branch. A tag that
  already exists makes the run fail: each commit has exactly one version.
- Promote without a rebuild: `gh release edit robot-v1.0.21 --prerelease=false`.
  Withdraw: `gh release delete robot-v1.0.21 --cleanup-tag`.
- The workflow doesn't produce `dev` releases (the version never contains "dev"); list those under
  `Releases:Channels:dev`.
- A release is only a label. Nothing builds or attaches packages yet, because the `build-debs` job has
  never run for real and there is no apt repo. The list says what *could* be pushed, not what is installable.

More on the version mapping and packaging: [robot/packaging/README.md](../robot/packaging/README.md).
