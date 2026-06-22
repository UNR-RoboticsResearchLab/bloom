#!/usr/bin/env bash

# Start script for development environment.
# Usage: ./startserver.sh [--migration] [--logs {frontend|backend|both}]
# If --migration is provided, the script will run EF Core migrations
# inside the built `server-dev` container after the database becomes ready.
# --logs option controls which container logs to tail (default: backend)

set -e

COMPOSE_FILE="docker-compose.yml"
SERVER_CONTAINER="bloom-server-dev"
REACT_CONTAINER="bloom-react-dev"
DB_SERVICE="mariadb-dev"
RUN_MIGRATIONS=false
LOGS_TARGET="backend"

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
  case $1 in
    --migration)
      RUN_MIGRATIONS=true
      shift
      ;;
    --logs)
      if [[ $2 =~ ^(frontend|backend|both)$ ]]; then
        LOGS_TARGET="$2"
        shift 2
      else
        echo "Error: --logs must be one of: frontend, backend, both"
        exit 1
      fi
      ;;
    *)
      echo "Error: Unknown argument: $1"
      echo "Usage: ./startserver-dev.sh [--migration] [--logs {frontend|backend|both}]"
      exit 1
      ;;
  esac
done

echo "Stopping any existing development containers..."
docker compose -f $COMPOSE_FILE down

echo "Starting Bloom development environment..."

# --- Start database first ---
echo "Starting database service: $DB_SERVICE..."
docker compose -f $COMPOSE_FILE up -d $DB_SERVICE

echo "Waiting for database ($DB_SERVICE:3306) to be ready..."
until docker-compose -f $COMPOSE_FILE logs $DB_SERVICE 2>&1 | grep -qi "ready for connections." || [ $SECONDS -gt 10 ]; do
  sleep 1
done
echo "Database is ready."

echo "Starting react container: $REACT_CONTAINER..."
docker compose -f $COMPOSE_FILE up -d $REACT_CONTAINER --build

# --- Start server ---
echo "Starting server container: $SERVER_CONTAINER..."
docker compose -f $COMPOSE_FILE up -d $SERVER_CONTAINER --build

# --- Optionally run migrations ---
if [ "$RUN_MIGRATIONS" = true ]; then
  echo "Running EF Core migrations inside $SERVER_CONTAINER..."
  docker exec $SERVER_CONTAINER dotnet ef database update || {
    echo "Migration failed or EF tools not installed."
  }
fi

# --- Tail logs for convenience ---
case $LOGS_TARGET in
  backend)
    echo "Attaching to backend logs (Ctrl+C to detach)..."
    docker logs -f $SERVER_CONTAINER
    ;;
  frontend)
    echo "Attaching to frontend logs (Ctrl+C to detach)..."
    docker logs -f $REACT_CONTAINER
    ;;
  both)
    echo "Attaching to both frontend and backend logs (Ctrl+C to detach)..."
    docker compose -f $COMPOSE_FILE logs -f $SERVER_CONTAINER $REACT_CONTAINER
    ;;
esac
