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
  echo "Restarting bloom.service..."
  sudo systemctl restart bloom.service
  sudo systemctl is-active --quiet bloom.service && echo "bloom.service is active" || {
    echo "Error: bloom.service failed to start. Check: sudo journalctl -u bloom.service -n 50"
    exit 1
  }
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
echo "Stopping bloom.service..."
sudo systemctl stop bloom.service || true

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

# --- Start the application via systemd ---
echo "Starting bloom.service..."
sudo systemctl start bloom.service
sleep 2
sudo systemctl is-active --quiet bloom.service && echo "bloom.service is active" || {
  echo "Error: bloom.service failed to start. Check: sudo journalctl -u bloom.service -n 50"
  exit 1
}

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
echo "Service: bloom.service"
echo "Logs: sudo journalctl -u bloom.service -f"
echo ""
echo "To restart: sudo systemctl restart bloom.service"
echo "To stop:    sudo systemctl stop bloom.service"
echo "Status:     sudo systemctl status bloom.service"
echo "================================"
