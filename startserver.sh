#!/usr/bin/env bash

# Start script for local development.
# Usage: ./startserver.sh [--migration]
# If --migration is provided, the script will run EF Core migrations
# inside the built `server-dev` container after the database becomes ready.

set -euo pipefail

DO_MIGRATE=0

# Simple args parsing: look for --migration
for arg in "$@"; do
    case "$arg" in
        --migration|--migrate)
            DO_MIGRATE=1
            ;;
        -h|--help)
            echo "Usage: $0 [--migration]"
            echo "  --migration   Run EF Core migrations against the mariadb-dev DB before starting the app"
            exit 0
            ;;
    esac
done

echo "Building docker images..."
docker-compose -f docker-compose.yml build

echo "Starting database container..."
docker-compose -f docker-compose.yml up -d mariadb-dev

# Wait for the database to be ready
echo "Waiting for the database to be ready..."

# Poll the mariadb-dev container logs until the server reports it's ready for connections.
# This avoids requiring mysqladmin/mysql client to be present in the server image.
until docker-compose -f docker-compose.yml logs mariadb-dev 2>&1 | grep -qi "ready for connections"; do
    sleep 2
done

echo "Database is ready."

if [ "$DO_MIGRATE" -eq 1 ]; then
    echo "Running EF Core migrations inside 'server-dev' container..."

    # Run the migrations in the server container. Use --no-deps so compose doesn't try to start
    # the mariadb-dev service again. We assume the image has dotnet and dotnet-ef available (see Dockerfile).
    docker-compose -f docker-compose.yml run --rm server-dev dotnet ef database update --no-build || {
        echo "Migration command failed. Exiting with non-zero status." >&2
        exit 1
    }

    echo "Migrations applied successfully."
fi

echo "Starting application containers..."
docker-compose -f docker-compose.yml up -d

echo "Bloom server started."