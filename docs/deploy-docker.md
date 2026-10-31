# Deploying the development branch to a server

Every push to `development` that passes the tests is deployed by GitHub Actions. A GitHub runner
builds the Docker image (with a layer cache) and pushes it to GitHub's container registry; the
deploy job then joins your Tailscale tailnet, SSHes to the server over it, checks out that commit,
and runs [deploy-docker.sh](../deploy-docker.sh), which **pulls** that image and restarts the stack
in [docker-compose.prod.yml](../docker-compose.prod.yml). The server never builds anything. The workflow is
[deploy-development.yml](../.github/workflows/deploy-development.yml); it calls
[ci.yml](../.github/workflows/ci.yml) for the tests, so `ci.yml` itself no longer runs on pushes to
`development` (pull requests into it still do).

```
push to development ─► tests ─► build image, push to ghcr.io/<owner>/bloom-server:<sha>
                              └► join tailnet ─► ssh server ─► git checkout <sha> ─► deploy-docker.sh
                                                  pull image ─► backup DBs ─► compose up ─► wait for HTTP 200
```

## The stack

| Service | What | Reachable from |
|---|---|---|
| `bloom-server` | ASP.NET Core API plus the built React app, `Production` mode | host loopback `127.0.0.1:5000` |
| `mariadb-prod` | Main database (migrations run when the server starts) | other containers only |
| `arsr-db` | ArSr study database | other containers only |
| `arsr-service` | Optional speech service, `--profile arsr`, built from a checkout outside this repo | other containers only |

It serves plain HTTP on loopback only. **Put a TLS reverse proxy in front** (nginx, Caddy, ...):
robot agents connect only over https, and the admin terminal needs the proxy to pass the `Host`
header and WebSocket upgrades. Caddy does both with `reverse_proxy 127.0.0.1:5000`.

## One-time server setup

1. **Container engine.** The server runs **podman** (rootless is fine); `deploy-docker.sh` uses
   podman when it is installed and docker otherwise (force one with `CONTAINER_ENGINE=docker`).
   Install `podman`, `git` and `curl`, plus a compose provider that `podman compose` can call:
   the `docker-compose` v2 binary (needs `systemctl --user enable --now podman.socket`) or
   `podman-compose`. Check with `podman compose version`. For a rootless deploy user also run, once:
   - `sudo loginctl enable-linger <user>`: without it, the containers stop when the last SSH session
     ends (every deploy ends with one).
   - `systemctl --user enable --now podman-restart.service`: podman, unlike docker, doesn't restart
     `restart: unless-stopped` containers after a reboot by itself.

   On SELinux hosts (Fedora, RHEL) the certificate mount carries `:z`, which relabels the file;
   nothing else is needed. Images are named in full (`docker.io/library/...`), so no registry
   configuration is needed either.
2. **Clone the repo** where `DEPLOY_PATH` will point, e.g. `~/bloom`:
   `git clone https://github.com/UNR-RoboticsResearchLab/bloom.git ~/bloom`
   (a private fork needs a read-only deploy key on the server).
3. **Secrets file.** `cp .env.example .env` and fill every value (`openssl rand -hex 24` for
   passwords, `openssl rand -base64 48` for `JWT_KEY`). Compose refuses to start with one missing.
   `.env` is gitignored; the `Jwt:Key` committed in `appsettings.json` is public and must never be
   used on a server.
4. **Data-protection certificate.** In Production the backend encrypts its cookie/key ring with a
   PKCS#12 file. It only protects keys at rest, so a self-signed one is fine:
   ```bash
   mkdir -p certs && cd certs
   openssl req -x509 -newkey rsa:2048 -nodes -keyout k.pem -out c.pem -days 3650 -subj "/CN=bloom-dataprotection"
   openssl pkcs12 -export -out bloomserver.pfx -inkey k.pem -in c.pem -passout pass:YOUR_DP_CERT_PASSWORD
   rm k.pem c.pem && chmod 600 bloomserver.pfx
   ```
   Use the same password for `DP_CERT_PASSWORD` in `.env`. Back this file up: without it the
   stored keys can't be read after a rebuild and everyone is logged out.
5. **Tailscale.** The server must be on your tailnet (`tailscale up`); the deploy job reaches it by
   its tailnet name or `100.x` address, so port 22 needn't be public. In the Tailscale admin
   console: define `tag:ci` under `tagOwners`, add an ACL that lets `tag:ci` reach the server on
   port 22, and create an OAuth client (Settings → OAuth clients) with the `auth_keys` write scope
   and the tag `tag:ci`. Keep its client ID and secret for step 7.
6. **Deploy key for GitHub Actions.** On any machine:
   `ssh-keygen -t ed25519 -N "" -C bloom-ci -f bloom-ci`, append `bloom-ci.pub` to the deploy
   user's `~/.ssh/authorized_keys`, and keep `bloom-ci` (private) for the next step.
7. **GitHub secrets.** Repo → Settings → Environments → New environment `development` (add
   required reviewers there if you want a manual approval gate), then add these secrets:

   | Secret | Value |
   |---|---|
   | `TS_OAUTH_CLIENT_ID`, `TS_OAUTH_SECRET` | the Tailscale OAuth client from step 5 |
   | `DEPLOY_HOST` | the server's tailnet name (MagicDNS) or `100.x` address |
   | `DEPLOY_USER` | the deploy user |
   | `DEPLOY_PATH` | absolute path of the checkout, e.g. `/home/deploy/bloom` |
   | `DEPLOY_SSH_KEY` | contents of the private key `bloom-ci` |
   | `DEPLOY_KNOWN_HOSTS` | output of `ssh-keyscan -t ed25519 <server>`, run from a tailnet machine. Only the `ssh-ed25519 AAAA…` key is used (it's pinned under `DEPLOY_HOST`), so the host name in the line doesn't matter. |

8. **Image registry and architecture.** Nothing to set up on the server: the deploy job hands it a
   short-lived token for the pull, and the server logs in, pulls and logs out. The image is
   `ghcr.io/<owner>/bloom-server:<commit>`. Two things to know:
   - **Architecture.** The image is built natively on the runner. If the server is **arm64**, add a
     repository variable (Settings → Secrets and variables → Actions → Variables) `DEPLOY_ARCH=arm64`
     so it builds on an arm runner; leave it unset for x86-64. A mismatch shows as `exec format error`
     or `no matching manifest`.
   - **Package access.** The first push creates the package. If the pull is denied, open the package
     (repo → Packages → bloom-server → Package settings), and under *Manage Actions access* add this
     repository with Read (or make the package public).
9. **First deploy.** Push to `development`, or run `./deploy-docker.sh` on the server by hand.
   Log in as `admin@example.com` with `SEED_ADMIN_PASSWORD`, then change it.

## Day to day

- **Logs:** `podman logs -f bloom-server` (`docker logs` if you use docker)
- **What's running:** `podman inspect --format '{{ index .Config.Labels "bloom.commit" }}' bloom-server`
- **Images on the server:** only `bloom-server` images are kept, the newest 3 (each deploy pulls a new tag).
  Other images on the machine are never touched.
- **Backups:** each deploy first dumps both databases to `~/bloom-backups` (newest 7 kept). Skip with
  `./deploy-docker.sh --no-backup`.
- **Roll back:** a failed deploy prints the exact command, which looks like
  `git checkout --detach <sha> && BLOOM_IMAGE=ghcr.io/<owner>/bloom-server:<sha> ./deploy-docker.sh --no-backup`.
  The old image is reused from the server if it is still there, otherwise pulled (that needs the
  registry reachable and, for a private package, a login: `podman login ghcr.io`). Migrations that already ran are
  not undone; restore a dump if a migration has to be reverted:
  `gunzip -c ~/bloom-backups/bloom_<time>.sql.gz | podman exec -i bloom-mariadb sh -c 'MYSQL_PWD="$MARIADB_ROOT_PASSWORD" mariadb -uroot'`.
- **Optional services:** `podman compose -f docker-compose.prod.yml --profile arsr up -d --build`
  (needs the AutoRSR source at `ARSR_SOURCE_DIR`, default `../AutoRsr-NoBert-SARG`).

## How the pipeline runs

| | |
|---|---|
| **Workflow** | [deploy-development.yml](../.github/workflows/deploy-development.yml) |
| **Starts on** | a push to `development`, or **Run workflow** by hand (it deploys the branch you pick, so run it on `development`) |
| **Job `tests`** | calls [ci.yml](../.github/workflows/ci.yml): backend tests (.NET 9) and frontend tests (Jest). If either fails, nothing deploys. |
| **Job `image`** | computes the version with `nbgv`; builds the [Dockerfile](../Dockerfile) with Buildx on a runner of the server's architecture (npm and NuGet layers are cached between runs in the GitHub cache); pushes `ghcr.io/<owner>/bloom-server:<commit>` |
| **Job `deploy`** | environment `development`; joins the tailnet as `tag:ci`; SSHes in with the pinned host key; runs `git fetch`, `git checkout --force --detach <sha>`, then `./deploy-docker.sh` from the new checkout with `BLOOM_IMAGE` set to that image and the job's registry token on stdin |
| **Concurrency** | one deploy at a time; a newer push waits instead of cancelling one half way |
| **Never deploys** | pull requests (they only run `ci.yml`), or any branch other than `development` |

`deploy-docker.sh` (run on the server) does, in order:

1. Picks the engine (podman, else docker), checks that `<engine> compose` works, logs in to the registry with the token from stdin (and logs out when done), then checks `.env`, the certificate and `compose config`, before anything is stopped.
2. **Pulls the image.** A missing image stops the deploy here, before anything is stopped or dumped. (If the registry can't be reached but the image is already on the server, as in a rollback, it carries on with that copy.)
3. Dumps both databases to `~/bloom-backups` (skipped on the very first deploy, when no database is running; a failed dump aborts the deploy).
4. `<engine> compose -f docker-compose.prod.yml up -d --no-build` with the pulled image. The commit sha is stored as the `bloom.commit` label on the container.
5. Polls `http://127.0.0.1:5000/` for up to 120 s. The app serves `index.html` itself and runs its
   migrations before it starts listening, so a 200 means the new version is up and migrated.
6. On success prunes dangling images and old `bloom-server` images; on failure prints the server's logs and the previous commit.

A manual `./deploy-docker.sh` on the server, with `BLOOM_IMAGE` unset, builds the image from the
checkout instead (slow; that is the old behaviour, kept for a server with no registry access). Its own checks (secrets present, config valid, 200
from the app) are what CI relies on, so a broken deploy shows up as a failed run.

## Troubleshooting

| Symptom (in the Actions log) | Cause and fix |
|---|---|
| `Secret X is not set in the 'development' environment` | Add it as an *environment* secret on `development`, not a repository secret. |
| Tailscale step fails or times out reaching the host | `tag:ci` isn't in `tagOwners`, the ACL doesn't allow `tag:ci` to the server on port 22, the OAuth client lacks the `auth_keys` write scope or the `tag:ci` tag, or the server is offline. |
| `DEPLOY_KNOWN_HOSTS has no ssh-ed25519 key` | The secret is empty, or holds another key type (RSA/ECDSA). Re-run `ssh-keyscan -t ed25519 <server>` from the tailnet and paste the output. |
| `Host key verification failed` (the key is now matched by value, not name) | The server's key changed (reinstalled, or `ssh-keygen -A` re-run), or `DEPLOY_HOST` points at a different machine than the one you scanned. Re-scan and update the secret. |
| `Permission denied (publickey)` | The public half of `DEPLOY_SSH_KEY` isn't in the deploy user's `authorized_keys`, or the secret is truncated (paste the whole file, including the `BEGIN`/`END` lines). |
| `git fetch` fails on the server | The clone can't reach GitHub (a private fork needs a read-only deploy key on the server). |
| `'podman compose' is not available` | No compose provider is installed. Install `docker-compose` v2 or `podman-compose` (step 1). |
| Everything works, then the site is down a while after a deploy or a reboot | Rootless podman: linger and `podman-restart.service` aren't enabled (step 1). |
| `permission denied` reading the certificate (SELinux) | The `:z` relabel didn't apply; check `ls -Z certs/`, or move the file to a path the deploy user owns. |
| `could not pull ghcr.io/...` / `denied` / `unauthorized` | The package isn't accessible to this repo's token: package settings → *Manage Actions access* → add the repo with Read, or make it public (setup step 8). |
| `exec format error`, or `no matching manifest for linux/...` | The server's architecture differs from the build's. Set the repository variable `DEPLOY_ARCH=arm64` for an arm64 server, unset it for x86-64, and push again. |
| `Build image` job fails on `type=gha` cache | Transient GitHub cache outage; re-run the job. The cache only speeds builds up. |
| `.env is missing` / `required variable ... is missing` | Fill `.env` from `.env.example` (step 3). |
| `data-protection certificate not found` | Create `certs/bloomserver.pfx` (step 4), or point `DP_CERT_FILE` at it. |
| `database backup failed; not deploying` | The database container is unhealthy or out of disk. Fix it, or rerun with `--no-backup` once you accept the risk. |
| `server did not come up` | Read the logs the script printed. Common causes: a wrong `DP_CERT_PASSWORD`, `SEED_ADMIN_PASSWORD` empty on a fresh database, a `;` in a database password, a failed migration. Then follow "Roll back" below. |
| Site loads over `http://localhost` but not through your domain | The TLS proxy isn't forwarding to `127.0.0.1:5000`, or doesn't pass the `Host` header and WebSocket upgrades. |

A fresh database logs "The model for context 'BloomDbContext' has pending changes. Add a new
migration". That is a known gap between the EF model and the migrations; the server starts anyway.

## Things to know

- **Tested with docker, not podman.** The compose file and `deploy-docker.sh` were run end to end
  under Docker (builds, a pull-based deploy from a local registry, a redeploy with backup, a bad-tag
  failure, the registry login and logout). I wrote the podman parts (engine detection,
  compose provider, linger, SELinux relabel) to avoid anything docker-specific, but they haven't run on
  a podman host; expect to adjust on the first deploy. Compose providers differ in what they support,
  so if `podman-compose` rejects the file, use the `docker-compose` v2 provider.
- Database passwords can't contain `;` (they are embedded in a connection string); hex is safe.
- CI builds the image, so deploys are quick once the layers are cached: a change that doesn't touch
  `frontend/package.json` or `backend/*.csproj` skips the `npm install` and NuGet restore.
- The image holds the build version (`nbgv`) and is tagged with the commit, so what runs on the
  server is exactly what the tests passed on.
- `RemoteShell__Enabled` defaults to on, matching `appsettings.json`. Set `REMOTE_SHELL_ENABLED=false`
  in `.env` if the server shouldn't offer the robot shell. It also refuses to start with a weak
  `JWT_KEY`.
- Only pushes to `development` deploy. Pull requests run the tests and never touch the server.
