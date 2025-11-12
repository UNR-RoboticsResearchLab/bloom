#!/usr/bin/env bash

# Start script for development environment.
# Usage: ./startserver.sh [--migration]
# If --migration is provided, the script will run EF Core migrations
# inside the built `server-dev` container after the database becomes ready.

set -e

COMPOSE_FILE="docker-compose.dev.yml"
SERVER_CONTAINER="bloom-server-dev"
REACT_CONTAINER="bloom-react-dev"
DB_SERVICE="mariadb-dev"
RUN_MIGRATIONS=false

# --- Parse arguments ---
if [ "$1" == "--migration" ]; then
  RUN_MIGRATIONS=true
fi

echo "Starting Bloom development environment..."

# --- Start database first ---
echo "Starting database service: $DB_SERVICE..."
docker compose -f $COMPOSE_FILE up -d $DB_SERVICE

echo "Waiting for database ($DB_SERVICE:3306) to be ready..."
until docker-compose -f docker-compose.dev.yml logs $DB_SERVICE 2>&1 | grep -qi "ready for connections"; do
    sleep 2
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

# --- Tail server logs for convenience ---
echo "Attaching to server logs (Ctrl+C to detach)..."
docker logs -f $REACT_CONTAINER
