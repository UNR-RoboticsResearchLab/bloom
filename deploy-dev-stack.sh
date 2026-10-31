#!/usr/bin/env bash
# Starts or updates the DEVELOPMENT stack from docker-compose.yml on a server: the hot-reload React dev
# server, the dev backend (Development mode, Swagger) and the two dev databases. This is separate from the
# production-mode stack that deploy-docker.sh runs; both can run side by side (different names, volumes, ports).
# Run it from the repo checkout, after the commit to run is checked out (CI does that over SSH).
#
# Usage: ./deploy-dev-stack.sh
# Reads .env (or the environment): DEV_BIND (required), DEV_DB_BIND, DEV_PUBLIC_ORIGINS, DEV_ADMIN_PASSWORD, and the
# host ports DEV_FRONTEND_PORT (3000), DEV_API_PORT (8080), DEV_HTTPS_PORT (2443), DEV_DB_PORT (3306), DEV_ARSR_DB_PORT (3307).
# See docs/deploy-docker.md, "The development stack on the server".
#
# Frontend source (frontend/src, frontend/public) is volume-mounted, so a checkout alone hot-reloads it. The
# backend image is rebuilt each time (cached layers, so only a changed backend/ or csproj costs time). The
# dev databases keep their data in named volumes across deploys. `auto_rsr` is not started: its source
# isn't in this repo and it needs an NVIDIA GPU.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
SERVICES=(mariadb-dev arsr-db-dev bloom-server-dev bloom-react-dev)
READY_TIMEOUT=300

die() { echo "ERROR: $*" >&2; exit 1; }

if [ -n "${CONTAINER_ENGINE:-}" ]; then ENGINE="$CONTAINER_ENGINE"
elif command -v podman >/dev/null; then ENGINE=podman
elif command -v docker >/dev/null; then ENGINE=docker
else die "neither podman nor docker is installed"; fi
command -v "$ENGINE" >/dev/null || die "$ENGINE is not installed"
COMPOSE=("$ENGINE" compose -f docker-compose.yml)
"${COMPOSE[@]}" version >/dev/null 2>&1 \
  || die "'$ENGINE compose' is not available: install docker-compose (v2) or podman-compose (docs/deploy-docker.md)"
command -v curl >/dev/null || die "curl is not installed"

# Values from .env (compose reads the same file itself); the environment wins.
env_value() { { grep -E "^$1=" .env 2>/dev/null || true; } | tail -n1 | cut -d= -f2-; }
DEV_BIND="${DEV_BIND:-$(env_value DEV_BIND)}"
DEV_ADMIN_PASSWORD="${DEV_ADMIN_PASSWORD:-$(env_value DEV_ADMIN_PASSWORD)}"
FRONTEND_PORT="${DEV_FRONTEND_PORT:-$(env_value DEV_FRONTEND_PORT)}"; FRONTEND_PORT="${FRONTEND_PORT:-3000}"
API_PORT="${DEV_API_PORT:-$(env_value DEV_API_PORT)}"; API_PORT="${API_PORT:-8080}"

# The dev stack is insecure by design (default database passwords, Swagger, dev certificates), and its
# ports default to every interface. Make the operator choose an address instead of exposing it by accident.
if [ -z "$DEV_BIND" ]; then
  die "DEV_BIND is not set in .env. Without it the dev stack is published on every network interface, with
the databases on default passwords. Set DEV_BIND to this server's Tailscale address (or 127.0.0.1) and, usually,
DEV_DB_BIND=127.0.0.1. See docs/deploy-docker.md."
fi
[ -n "$DEV_ADMIN_PASSWORD" ] \
  || echo "WARNING: DEV_ADMIN_PASSWORD is not set; a new dev database gets admin@example.com / Admin@123." >&2

"${COMPOSE[@]}" config >/dev/null || die "docker-compose.yml is not valid with this .env"

COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "Deploying the dev stack at $COMMIT (${SERVICES[*]})"
"${COMPOSE[@]}" up -d --build "${SERVICES[@]}"

# Probe the address the ports are published on (0.0.0.0 means any interface; loopback reaches it).
HOST="$DEV_BIND"; [ "$HOST" = "0.0.0.0" ] && HOST=127.0.0.1
check() { [ "$(curl -s -o /dev/null -w '%{http_code}' "$1" || true)" = "200" ]; }

echo "Waiting up to ${READY_TIMEOUT}s for the backend (:$API_PORT) and the React dev server (:$FRONTEND_PORT) on $HOST ..."
deadline=$((SECONDS + READY_TIMEOUT))
until check "http://$HOST:$API_PORT/swagger/index.html" && check "http://$HOST:$FRONTEND_PORT/"; do
  if [ "$SECONDS" -ge "$deadline" ]; then
    for c in bloom-server-dev bloom-react-dev; do
      echo "--- $c logs ---" >&2
      "$ENGINE" logs --tail 40 "$c" >&2 || true
    done
    die "the dev stack did not come up"
  fi
  sleep 5
done

echo "Dev stack up at $COMMIT:  frontend http://$HOST:$FRONTEND_PORT   API/Swagger http://$HOST:$API_PORT/swagger/index.html"
