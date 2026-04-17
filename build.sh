#!/usr/bin/env bash
# Production deployment script for Bloom (ASP.NET Core + React)
# Usage: ./build.sh [--migration] [--backup] [--skip-build]

set -euo pipefail

# Configuration
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_PORT=${SERVER_PORT:-5000}
DB_HOST=${DB_HOST:-localhost}
DB_PORT=${DB_PORT:-3306}
DB_NAME=${DB_NAME:-bloomdb}
DOTNET_ENV="Production"
RUN_MIGRATIONS=false
BACKUP_DB=false
SKIP_BUILD=false
LOG_DIR="$PROJECT_ROOT/logs"

# --- Parse arguments ---
for arg in "$@"; do
  case $arg in
    --migration)
      RUN_MIGRATIONS=true
      shift
      ;;
    --backup)
      BACKUP_DB=true
      shift
      ;;
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    *)
      ;;
  esac
done

echo "=================================="
echo "   Bloom Production Deployment"
echo "=================================="

# --- Ensure required tools are available ---
for tool in dotnet node mysql npm; do
  if ! command -v "$tool" &> /dev/null; then
    echo "Error: $tool not found. Please install it before running this script."
    exit 1
  fi
done

# --- Check database connectivity ---
echo "Checking database connectivity..."
if ! mysql -h "$DB_HOST" -P "$DB_PORT" -u jenkins -p"${DB_PASSWORD:-}" -e "SELECT 1" &> /dev/null; then
  echo "Error: Cannot connect to database at $DB_HOST:$DB_PORT"
  echo "Please ensure MariaDB/MySQL is running and accessible."
  exit 1
fi
echo "Database is accessible."

# --- Backup database if requested ---
if [ "$BACKUP_DB" = true ]; then
  echo "Creating database backup..."
  BACKUP_FILE="$PROJECT_ROOT/backups/bloom_$(date +%Y%m%d_%H%M%S).sql"
  mkdir -p "$PROJECT_ROOT/backups"
  if mysqldump -h "$DB_HOST" -P "$DB_PORT" -u root -p"${DB_PASSWORD:-}" "$DB_NAME" > "$BACKUP_FILE"; then
    echo "Database backed up to $BACKUP_FILE"
  else
    echo "Warning: Database backup failed."
  fi
fi

# --- Run migrations if requested ---
if [ "$RUN_MIGRATIONS" = true ]; then
  echo "Running EF Core migrations..."
  cd "$PROJECT_ROOT"
  if dotnet ef database update --configuration Release; then
    echo "Migrations applied successfully."
  else
    echo "Error: Migration failed."
    exit 1
  fi
fi

# --- Build React frontend ---
if [ "$SKIP_BUILD" = false ]; then
  echo "Building React frontend..."
  cd "$PROJECT_ROOT/ClientApp"
  if npm install && npm run build; then
    echo "React frontend built successfully."
  else
    echo "Error: React build failed."
    exit 1
  fi
fi

# --- Build .NET backend ---
if [ "$SKIP_BUILD" = false ]; then
  echo "Building .NET backend..."
  cd "$PROJECT_ROOT"
  if dotnet publish -c Release -o /var/www/bloom-build/net9.0/publish; then
    echo ".NET backend built successfully."
  else
    echo "Error: .NET build failed."
    exit 1
  fi
fi

# --- Stop existing server process ---
echo "Stopping existing server process..."
if pkill -f "dotnet.*bloom" || true; then
  sleep 2
  echo "Previous server process stopped."
fi



# --- Show deployment summary ---
echo ""
echo "================================"
echo "Build Complete"
echo "================================"