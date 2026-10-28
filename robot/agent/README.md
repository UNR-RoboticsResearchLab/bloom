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
2. **Register the robot** as an admin. The response contains the robot's `id` and its `secret`.
   The secret is shown only this once.
   ```bash
   curl -b cookies -H 'Content-Type: application/json' \
     -d '{"name":"blossom-07","model":"Blossom","ipAddress":"N/A"}' https://bloom.example.edu/api/robot/register
   ```
   For a robot that already exists, `POST /api/robot/{id}/credentials` issues a new secret.
3. **On the robot**, from the extracted tarball:
   ```bash
   sudo ./install.sh --backend-url https://bloom.example.edu --robot-id <id> --secret-file - <<< '<secret>'
   ```
   The script:
   - creates `bloom-ops`;
   - installs to `/opt/bloom-agent/<version>`, pointed to by a `current` symlink;
   - writes `/etc/bloom-agent/agent.json` and `/etc/bloom-agent/secret` (0600);
   - enables `bloom-agent.service`;
   - runs `doctor`.

   Re-running it upgrades the agent and keeps the existing config.
4. `bloom_node` uses the same id and secret. Put them in `robot/.env` as `BLOOM_ROBOT_ID` and
   `BLOOM_ROBOT_SECRET`.
5. Once the robot shows as online in the fleet view and a shell works, you can close sshd with
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

## Revoking a robot

```bash
curl -b cookies -X POST https://bloom.example.edu/api/robot/<id>/revoke
```

This takes effect immediately:
- the robot's secret is deleted;
- every token it holds stops working on the next request (`TokenVersion` is bumped);
- the live tunnel and any open shell are closed with reason `revoked`.

After that, the agent and `bloom_node` get `401` on every login and keep backing off. To bring
the robot back, issue new credentials with `POST /api/robot/<id>/credentials` and write the new
secret on the robot:

```bash
sudo tee /etc/bloom-agent/secret >/dev/null <<< '<new secret>'   # the agent picks it up on its next login
```

Also update `BLOOM_ROBOT_SECRET` in `robot/.env` and restart `bloom.service`. Rotating credentials
also drops the live tunnel, so it doubles as "kick everyone off this robot".

## Wire protocol

This is in `src/protocol.h`, mirrored in `backend/Services/RemoteShell/ShellProtocol.cs`.
- Control messages are JSON text frames: `hello`, `ping`/`pong`, `open`, `resize`, `close`,
  `exited`.
- Terminal bytes are binary frames: `[0x01][16-byte session id][payload]`.
- The version in `hello` is SemVer 2.0. The backend stores it only if it parses, and compares it
  as SemVer, never as a string. OTA in Phase 2 relies on that.
