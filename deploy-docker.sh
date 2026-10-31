#!/usr/bin/env bash
# Deploys the checked-out commit with docker-compose.prod.yml. Run on the server, from the repo
# checkout, after the commit to deploy is checked out (CI does that over SSH: see
# deploy-development.yml). Uses podman if it is installed, else docker.
#
# Usage: ./deploy-docker.sh [--no-backup]
# Environment: BUILD_VERSION (nbgv SemVer, stamped into the build; default 0.0.0-local)
#              CONTAINER_ENGINE (podman or docker; default: podman if present, else docker)
#
# What it does: checks the secrets and certificate exist, dumps the databases (kept: last 7), builds
# and restarts the stack, waits for the server to answer, and on failure prints the logs and the
# commit to go back to. Database migrations run by themselves when the server starts.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
BACKUP_DIR="${BACKUP_DIR:-$HOME/bloom-backups}"
KEEP_BACKUPS=7
READY_TIMEOUT=120
DO_BACKUP=true
[ "${1:-}" = "--no-backup" ] && DO_BACKUP=false

die() { echo "ERROR: $*" >&2; exit 1; }

if [ -n "${CONTAINER_ENGINE:-}" ]; then ENGINE="$CONTAINER_ENGINE"
elif command -v podman >/dev/null; then ENGINE=podman
elif command -v docker >/dev/null; then ENGINE=docker
else die "neither podman nor docker is installed"; fi
command -v "$ENGINE" >/dev/null || die "$ENGINE is not installed"
COMPOSE=("$ENGINE" compose -f docker-compose.prod.yml)
"${COMPOSE[@]}" version >/dev/null 2>&1 \
  || die "'$ENGINE compose' is not available: install docker-compose (v2) or podman-compose (docs/deploy-docker.md)"
command -v curl >/dev/null || die "curl is not installed"
[ -f .env ] || die ".env is missing. Copy .env.example to .env and fill it in (docs/deploy-docker.md)."

# Values from .env that this script needs (compose reads the same file itself).
env_value() { grep -E "^$1=" .env | tail -n1 | cut -d= -f2- || true; }
CERT_FILE="$(env_value DP_CERT_FILE)"; CERT_FILE="${CERT_FILE:-./certs/bloomserver.pfx}"
PORT="$(env_value BLOOM_HTTP_PORT)"; PORT="${PORT:-5000}"
[ -f "$CERT_FILE" ] || die "data-protection certificate not found at $CERT_FILE (docs/deploy-docker.md, step 4)"

# Catches a missing required value before anything is stopped.
"${COMPOSE[@]}" config >/dev/null || die "docker-compose.prod.yml is not valid with this .env"

PREVIOUS="$("$ENGINE" inspect --format '{{ index .Config.Labels "bloom.commit" }}' bloom-server 2>/dev/null || true)"
COMMIT="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
echo "Deploying $COMMIT (version ${BUILD_VERSION:-0.0.0-local}); running: ${PREVIOUS:-nothing}"

# Containers are addressed by their fixed names, so this doesn't depend on the compose provider.
if $DO_BACKUP && [ -n "$("$ENGINE" ps -q --filter name='^bloom-mariadb$' 2>/dev/null)" ]; then
  mkdir -p "$BACKUP_DIR"
  file="$BACKUP_DIR/bloom_$(date +%Y%m%d_%H%M%S).sql.gz"
  echo "Backing up the databases to $file"
  # Password comes from the container's own environment, so it never appears on a command line.
  if "$ENGINE" exec -i bloom-mariadb sh -c 'MYSQL_PWD="$MARIADB_ROOT_PASSWORD" mariadb-dump -uroot --single-transaction --all-databases' | gzip > "$file"; then
    # Prune: keep the newest $KEEP_BACKUPS.
    ls -1t "$BACKUP_DIR"/bloom_*.sql.gz | tail -n +$((KEEP_BACKUPS + 1)) | xargs -r rm -f --
  else
    rm -f "$file"
    die "database backup failed; not deploying. Fix it, or rerun with --no-backup."
  fi
fi

export BUILD_VERSION="${BUILD_VERSION:-0.0.0-local}"
export BLOOM_COMMIT="$COMMIT"
"${COMPOSE[@]}" up -d --build

echo "Waiting up to ${READY_TIMEOUT}s for the server on 127.0.0.1:$PORT ..."
# The SPA's index.html is served by the app itself, so a 200 means Kestrel is up and migrations ran
# (they run before the server starts listening).
deadline=$((SECONDS + READY_TIMEOUT))
until [ "$(curl -s -o /dev/null -w '%{http_code}' "http://127.0.0.1:$PORT/" || true)" = "200" ]; do
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "--- bloom-server logs ---" >&2
    "$ENGINE" logs --tail 80 bloom-server >&2 || true
    echo >&2
    [ -n "$PREVIOUS" ] && echo "To go back: git checkout --detach $PREVIOUS && ./deploy-docker.sh --no-backup" >&2
    echo "A migration may already have run; the pre-deploy dump is in $BACKUP_DIR." >&2
    die "server did not come up"
  fi
  sleep 3
done

"$ENGINE" image prune -f >/dev/null
echo "Deployed $COMMIT."
