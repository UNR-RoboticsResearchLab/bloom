# robot-agent

Gives admins a shell on a robot from the fleet dashboard, with no inbound ports. The agent keeps
one **outbound** TLS WebSocket open to the backend (port 443 by default, through the system HTTP
proxy if there is one). The backend relays the admin's browser terminal over it into a PTY on the
robot.

```
Browser (xterm.js) ──wss, admin cookie──▶ Backend relay ◀──wss, robot JWT── robot-agent ──▶ bash on a PTY
```

- Plain CMake, C++17, **not** a ROS 2 package (`COLCON_IGNORE`). It's how you get in when the
  ROS stack is broken, so it must not depend on it.
- Shells run as the dedicated user `bloom-ops`, which has passwordless sudo. Its password is
  locked, so the agent is the only way to log in as it.
- Every session is audited by the backend: who opened it, which robot, when it started and ended,
  and why it ended. Keystrokes and output are never stored.

## Build

```bash
sudo apt install cmake g++ libboost-dev libssl-dev libgtest-dev   # Ubuntu 24.04
cmake -S robot/agent -B robot/agent/build
cmake --build robot/agent/build -j
ctest --test-dir robot/agent/build
```

Local builds are versioned `0.0.0-local+<commit>`. CI (`.github/workflows/robot-agent.yml`) builds
the release artifact on a native arm64 runner and stamps it with the same Nerdbank.GitVersioning
version as the backend (`1.0.<git height>`, plus a `-g<commit>` suffix off `main`). It produces
`robot-agent-<version>.tar.gz` and a `.sha256`. The binary links the system OpenSSL, so it targets
Ubuntu 24.04 specifically, not any arm64 Linux.

## Provision and install a robot

1. **Turn the shell on in the backend** (it's off by default). Set `RemoteShell__Enabled=true`.
   It won't start unless `Jwt__Key` is a private key of at least 32 bytes; the one in
   `appsettings.json` is public. See `backend/.env.example`.
2. **Add the robot** in the dashboard (Fleet → **Add robot**). It shows the robot's ID and secret
   once and offers a **provisioning file** to download (`<name>.provision.json`: backend URL, robot
   ID, name, secret; it holds the secret in plain text, so delete it after use). From a shell, the
   equivalent is `POST /api/robot/register`.
3. **On the robot**, from the extracted tarball, with the provisioning file copied over (USB stick or
   scp):
   ```bash
   sudo ./install.sh --provision-file blossom-07.provision.json
   ```
   The script:
   - creates `bloom-ops`;
   - installs to `/opt/bloom-agent/<version>`, pointed to by a `current` symlink;
   - applies the credentials with `provision.sh` (below), which also updates `bloom_node`'s `.env`;
   - enables `bloom-agent.service`;
   - runs `doctor`.

   Re-running it upgrades the agent and keeps the existing config. Without the file you can pass
   `--backend-url`, `--robot-id` and `--secret-file -` instead.
4. Once the robot shows as online in the fleet view and a shell works, you can close sshd with
   `sudo ./install.sh --disable-sshd`. The script refuses while `doctor` is failing, so you can't
   lock yourself out.

Logs: `journalctl -u bloom-agent -f`. Warnings and errors only: `journalctl -u bloom-agent -p warning`.

## `robot-agent doctor`

Run it on a robot on an unfamiliar network. It checks each layer in turn and prints a hint for
whatever breaks:

```
$ sudo -u bloom-ops /opt/bloom-agent/current/robot-agent doctor
  ✓ Config             /etc/bloom-agent/agent.json: backend https://bloom.example.edu
  ✓ Credentials        robot 7f2d…, secret present
  ✓ Proxy              via proxy.uni.edu:3128 (from HTTPS_PROXY)
  ✓ DNS                proxy.uni.edu -> 10.1.2.3
  ✓ Proxy CONNECT      connected to bloom.example.edu
  ✗ TLS certificate    not trusted: self-signed certificate in certificate chain
                       hint: this network appears to intercept TLS ...
```

| Check | What it looks for |
|---|---|
| Config, Credentials | Config parses; robot id and secret are present |
| Proxy | Which proxy is used for the backend, if any (`HTTPS_PROXY`/`https_proxy`/`ALL_PROXY`, `NO_PROXY`) |
| DNS, TCP / Proxy CONNECT | Name resolution and reachability: directly, or via the proxy's CONNECT |
| TLS certificate / issuer | Chain and hostname verify. If they don't, it prints the certificate it was given. TLS interception (campus inspection appliances) shows up here. Set `expected_issuers` to also flag certificates from an unexpected CA |
| Captive portal | `captive_check_url` (Ubuntu's connectivity check) must answer HTTP 204 |
| Backend | `GET /api/agent/health` must come back from the Bloom backend, not a portal or proxy error page |
| Login, WebSocket | The credentials are accepted and `/ws/robot-agent` upgrades. Doctor never sends `hello`, so it doesn't disturb the running agent |

If the network inspects TLS and policy requires it, add the inspection CA with `"ca_file"` in
`agent.json`. That file is used in addition to the system store.

## Connection behaviour

- The agent logs in fresh (`POST /api/robot/login`) before every connection.
- Reconnects use full-jitter exponential backoff: 1 s base, 60 s cap. It never gives up, and it
  resets after a tunnel has stayed up for 60 s.
- The agent sends an app-level `ping` every 25 s. A link silent for 60 s is treated as dead and
  reconnected. The backend drops agents that are silent for 75 s.
- **The backend closes every tunnel when its JWT expires** (`Jwt:ExpireMinutes`, 30 by default).
  The agent reconnects immediately, but **any open shell on that robot ends with
  `token_expired`**.
- When a tunnel drops, every shell is hung up. Everything on the terminal gets SIGHUP, as with
  ssh, so a job started with `nohup` survives. Stopping the service kills everything it started.

## Changing a robot's credentials

`bloom_node` and the agent share one ID and secret, so changing it means updating both on the robot.
That is what `deploy/provision.sh` does, **all-or-nothing** (any failure restores every file):

| File | Change |
|---|---|
| `/etc/bloom-agent/secret` | the new secret (0600, owned by `bloom-ops`) |
| `/etc/bloom-agent/agent.json` | `backend_url`, `robot_id` (file mode only) |
| `bloom_node`'s `.env` | `BLOOM_ROBOT_ID` / `BLOOM_ROBOT_SECRET`, other keys untouched. Found from `EnvironmentFile=` in `/etc/systemd/system/bloom.service`, or pass `--env-file` |

It then restarts `bloom-agent` and `bloom`. There are two ways to run it, both from the dashboard's
Fleet → robot → **Credentials** button:

- **Push** (robot online). The backend sends the new secret to the agent through its live tunnel; the
  agent runs `sudo -n /opt/bloom-agent/current/provision.sh --secret-stdin --no-restart-agent` with the
  secret on stdin and confirms. The backend then drops the tunnel and the agent reconnects with the
  new secret. The secret never passes through the admin's browser.
  *Nothing can lock the robot out:* the new secret is stored as **pending** and the old one keeps
  working until the robot first logs in with the new one. If the push times out or the connection
  drops, both secrets work. If the agent reports a failure (e.g. `sudo` isn't allowed), the pending
  secret is discarded and nothing changed. Needs the install.sh setup (`bloom-ops` with passwordless
  sudo) and **doesn't work if the secret comes from systemd `LoadCredential`**.
- **Provisioning file** (robot offline, or a push failed). The old secret stops working immediately.
  Download the file, copy it to the robot, and run:
  ```bash
  sudo ./provision.sh blossom-07.provision.json --remove     # --remove deletes the file afterwards
  ```
  (`provision.sh` is installed next to the binary, at `/opt/bloom-agent/current/provision.sh`.)

## Deleting a robot

Fleet → robot → **Delete**, and type the robot's name to confirm (`DELETE /api/robot/{id}`, admin
only). The robot can no longer log in, and if it's online its tunnel and any open shell are closed. Its
shell audit history is kept.

## Revoking a robot

Revoking keeps the robot registered but locks it out until you re-issue credentials:

```bash
curl -b cookies -X POST https://bloom.example.edu/api/robot/<id>/revoke
```

This takes effect immediately:
- the robot's secret is deleted;
- every token it holds stops working on the next request (`TokenVersion` is bumped);
- the live tunnel and any open shell are closed with reason `revoked`.

After that, the agent and `bloom_node` get `401` on every login and keep backing off. To bring the
robot back, use **Credentials → Download a provisioning file** (a push can't reach an offline,
revoked robot) and apply it with `provision.sh`. Issuing a new secret that way also drops the live
tunnel, so it doubles as "kick everyone off this robot".

## Wire protocol

This is in `src/protocol.h`, mirrored in `backend/Services/RemoteShell/ShellProtocol.cs`.
- Control messages are JSON text frames: `hello`, `ping`/`pong`, `open`, `resize`, `close`,
  `exited`.
- Terminal bytes are binary frames: `[0x01][16-byte session id][payload]`.
- The version in `hello` is SemVer 2.0. The backend stores it only if it parses, and compares it
  as SemVer, never as a string. OTA in Phase 2 relies on that.
