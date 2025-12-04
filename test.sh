#!/usr/bin/env bash
# Test script for Bloom project
# Usage: ./test.sh [--prod]
# Runs automated tests for React and Server containers.
# Designed for local and CI (Jenkins) environments.

set -euo pipefail

COMPOSE_FILE="docker-compose.dev.yml"
SERVER_CONTAINER="bloom-server-dev"
REACT_CONTAINER="bloom-react-dev"
DB_SERVICE="mariadb-dev"

# --- Parse arguments ---
for arg in "$@"; do
  case $arg in
    --prod)
      COMPOSE_FILE="docker-compose.yml"
      SERVER_CONTAINER="bloom-server"
      REACT_CONTAINER="bloom-react"
      DB_SERVICE="mariadb-prod"
      shift
      ;;
    *)
      ;;
  esac
done

echo "Running tests for Bloom environment using $COMPOSE_FILE..."

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

# --- Build app containers ---
echo "Building $SERVER_CONTAINER and $REACT_CONTAINER for testing..."
docker compose -f "$COMPOSE_FILE" build "$SERVER_CONTAINER" "$REACT_CONTAINER"

# --- Run tests ---
TEST_FAILED=false

echo "Running server tests in $SERVER_CONTAINER..."
if ! docker compose -f "$COMPOSE_FILE" run --rm "$SERVER_CONTAINER" dotnet test --logger "trx;LogFileName=test_results.trx"; then
  echo "Server tests failed."
  TEST_FAILED=true
fi

echo "Running React tests in $REACT_CONTAINER..."
if ! docker compose -f "$COMPOSE_FILE" run --rm "$REACT_CONTAINER" npm test -- --ci --watchAll=false; then
  echo "React tests failed."
  TEST_FAILED=true
fi

# --- Tear down containers ---
echo "Cleaning up containers..."
docker compose -f "$COMPOSE_FILE" down -v

if [ "$TEST_FAILED" = true ]; then
  echo "One or more test suites failed."
  exit 1
fi

echo "All tests passed successfully!"
