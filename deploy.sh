#!/usr/bin/env bash
# Production deployment script for Bloom (non-Docker)
# Deploys application to target directory and manages the running service
# Usage: ./deploy.sh [--prod] [--migration] [--restart-only]
# made w claude

REQUIRED_USER="www-data"

if [ "$(id -un)" != "$REQUIRED_USER" ]; then
  echo "ERROR: This deploy script must be run as $REQUIRED_USER"
  echo "Run it with: sudo -u $REQUIRED_USER $0 $*"
  exit 1
fi

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PUBLISH_DIR="$(ls -d "/var/www/bloom-build/net9.0/publish" 2>/dev/null | head -n 1)"
FRONTEND_BUILD_DIR="${FRONTEND_BUILD_DIR:-$SCRIPT_DIR/ClientApp/build}"
TARGET_DIR=""
RUN_MIGRATIONS=false
RESTART_ONLY=false
APP_PORT=5000


# --- Parse arguments ---
for arg in "$@"; do
  case $arg in
    --prod)
      TARGET_DIR="/var/www/bloom"
      ;;
    --migration)
      RUN_MIGRATIONS=true
      ;;
    --restart-only)
      RESTART_ONLY=true
      ;;
    *)
      ;;
  esac
done

for tool in dotnet curl; do
  command -v "$tool" &>/dev/null || {
    echo "Error: $tool is required but not installed."
    exit 1
  }
done


# Default to dev target if none provided
if [ -z "$TARGET_DIR" ]; then
  TARGET_DIR="/var/www/bloom-dev"
fi

echo "================================"
echo "Bloom Application Deployment"
echo "================================"
echo "Target directory: $TARGET_DIR"
echo ""

# --- Check if restart-only mode ---
if [ "$RESTART_ONLY" = true ]; then
  echo "Restarting application in $TARGET_DIR..."

  if [ ! -f "$TARGET_DIR/app.pid" ]; then
    echo "No running application found (app.pid not found)"
    exit 1
  fi

  OLD_PID=$(cat "$TARGET_DIR/app.pid")
  if kill -0 "$OLD_PID" 2>/dev/null; then
    echo "Stopping previous application (PID: $OLD_PID)..."
    kill "$OLD_PID" || true
    sleep 2
  fi
  rm -f "$TARGET_DIR/app.pid"

  # Start the application
  cd "$TARGET_DIR"
  nohup dotnet bloom.dll > "$TARGET_DIR/logs/app.log" 2>&1 &
  NEW_PID=$!
  echo $NEW_PID > "$TARGET_DIR/app.pid"
  echo "Application started (PID: $NEW_PID)"
  exit 0
fi

# --- Ensure build output exists ---
if [ -z "$PUBLISH_DIR" ]; then
  echo "Error: No publish directory found!"
  echo "Please run ./build.sh first to build the application."
  exit 1
fi

echo "Using publish directory: $PUBLISH_DIR"


echo "Preparing deployment..."

# --- Stop existing application ---
if [ -f "$TARGET_DIR/app.pid" ]; then
  OLD_PID=$(cat "$TARGET_DIR/app.pid")
  if kill -0 "$OLD_PID" 2>/dev/null; then
    echo "Stopping previous application (PID: $OLD_PID)..."
    kill "$OLD_PID" || true
    sleep 2
  fi
fi

# --- Create target directory structure ---
echo "Setting up target directory: $TARGET_DIR"
mkdir -p "$TARGET_DIR"
find "$TARGET_DIR" -mindepth 1 -maxdepth 1 ! -name logs ! -name backups -exec rm -rf {} +
mkdir -p "$TARGET_DIR"
mkdir -p "$TARGET_DIR/logs"
mkdir -p "$TARGET_DIR/backups"
mkdir -p "$TARGET_DIR/build"

# --- Deploy application files ---
echo "Deploying application files..."
cp -r "$PUBLISH_DIR/"* "$TARGET_DIR/"

# --- Deploy frontend files ---
echo "Deploying frontend files..."
if [ ! -d "$FRONTEND_BUILD_DIR" ]; then
  echo "Error: Frontend build directory not found at $FRONTEND_BUILD_DIR"
  echo "Please run ./build.sh first to build the React frontend."
  exit 1
fi
cp -r "$FRONTEND_BUILD_DIR/"* "$TARGET_DIR/build"



# --- Run migrations if requested ---
if [ "$RUN_MIGRATIONS" = true ]; then
  echo "Running database migrations..."
  cd "$TARGET_DIR"
  if dotnet bloom.dll --run-migrations; then
    echo "Migrations completed successfully."
  else
    echo "Warning: Migrations may have failed. Check logs."
  fi
fi

# --- Start the application ---
echo "Starting Bloom application..."
cd "$TARGET_DIR"
nohup dotnet bloom.dll --urls "http://localhost:$APP_PORT" > "$TARGET_DIR/logs/app.log" 2>&1 &
APP_PID=$!
echo $APP_PID > "$TARGET_DIR/app.pid"

echo "Application started with PID: $APP_PID"

# --- Wait for application to be ready ---
echo "Waiting for application to become ready..."
# ATTEMPTS=0
# until curl -s http://localhost:$APP_PORT/health &> /dev/null; do
#   sleep 2
#   ((ATTEMPTS++))
#   if [ "$ATTEMPTS" -gt 30 ]; then
#     echo "Warning: Application may not have started properly."
#     echo "Check logs at: $TARGET_DIR/logs/app.log"
#     break
#   fi
# done

# --- Deployment summary ---
echo ""
echo "================================"
echo "Deployment Complete"
echo "================================"
echo "Target directory: $TARGET_DIR"
echo "Application PID: $APP_PID"
echo "Logs: $TARGET_DIR/logs/app.log"
echo ""
echo "To restart: ./deploy.sh $([ "$TARGET_DIR" = "/var/www/bloom" ] && echo "--prod" || echo "") --restart-only"
echo "To stop: kill $APP_PID"
echo "================================"
