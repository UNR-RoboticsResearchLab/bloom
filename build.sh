#!/usr/bin/env bash
# Build and start script for development or CI environments.
# Usage: ./build.sh [--migration] [--prod]

set -euo pipefail

COMPOSE_FILE="docker-compose.dev.yml"
SERVER_CONTAINER="bloom-server-dev"
REACT_CONTAINER="bloom-react-dev"
DB_SERVICE="mariadb-dev"
RUN_MIGRATIONS=false

# --- Parse arguments ---
for arg in "$@"; do
  case $arg in
    --migration)
      RUN_MIGRATIONS=true
      shift
      ;;
    --prod)
      COMPOSE_FILE="docker-compose.yml"
      SERVER_CONTAINER="bloom-server"
      DB_SERVICE="mariadb-prod"
      shift
      ;;
    *)
      ;;
  esac
done

echo "Starting Bloom environment using $COMPOSE_FILE..."

# --- Ensure Docker is available ---
if ! command -v docker &> /dev/null; then
  echo "Docker not found. Please install Docker before running this script."
  exit 1
fi

# --- Start database first ---
echo "Starting database service: $DB_SERVICE..."
docker compose -f "$COMPOSE_FILE" up -d "$DB_SERVICE"

# --- Wait for DB to be ready ---
echo "Waiting for $DB_SERVICE (port 3306) to be ready..."
ATTEMPTS=0
until docker compose -f "$COMPOSE_FILE" logs "$DB_SERVICE" 2>&1 | grep -qi "ready for connections"; do
  sleep 2
  ((ATTEMPTS++))
  if [ "$ATTEMPTS" -gt 30 ]; then
    echo "Database did not become ready in time."
    docker compose -f "$COMPOSE_FILE" logs "$DB_SERVICE" | tail -n 20
    exit 1
  fi
done
echo "Database is ready."

# --- Build and start React + Server ---
echo "Building and starting $REACT_CONTAINER and $SERVER_CONTAINER..."
docker compose -f "$COMPOSE_FILE" up -d --build "$REACT_CONTAINER" "$SERVER_CONTAINER"

# --- Optionally run migrations ---
if [ "$RUN_MIGRATIONS" = true ]; then
  echo "Running EF Core migrations inside $SERVER_CONTAINER..."
  if docker exec "$SERVER_CONTAINER" dotnet ef database update; then
    echo "Migrations applied successfully."
  else
    echo "Migration failed or EF tools missing."
  fi
fi

# --- Tail server logs if running locally ---
if [[ -t 1 ]]; then
  echo "Attaching to $SERVER_CONTAINER logs (Ctrl+C to detach)..."
  docker logs -f "$SERVER_CONTAINER"
else
  echo "Build and startup complete (non-interactive environment)."
fi