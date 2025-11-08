set -euo pipefail

TARGET_DIR=""
COMPOSE_FILE="docker-compose.dev.yml"
SERVER_CONTAINER="bloom-server-dev"
RUN_MIGRATIONS=false

# --- Parse arguments ---
for arg in "$@"; do
  case $arg in
    --prod)
      TARGET_DIR="/var/www/project"
      COMPOSE_FILE="docker-compose.yml"
      SERVER_CONTAINER="bloom-server"
      ;;
    --migration)
      RUN_MIGRATIONS=true
      ;;
    *)
      ;;
  esac
done

# Default to dev target if none provided
if [ -z "$TARGET_DIR" ]; then
  TARGET_DIR="/var/www/project-dev"
fi

echo "🚀 Deploying Bloom application to $TARGET_DIR"
echo "Using compose file: $COMPOSE_FILE"

# --- Ensure build output exists ---
if [ ! -d "build" ] && [ ! -d "publish" ]; then
  echo "No build or publish directory found! Did you run build.sh?"
  exit 1
fi

BUILD_SRC=""
if [ -d "build" ]; then
  BUILD_SRC="build"
elif [ -d "publish" ]; then
  BUILD_SRC="publish"
fi

# --- Ensure target directory exists ---
echo "Preparing target directory: $TARGET_DIR"
sudo mkdir -p "$TARGET_DIR"
sudo chown -R "$USER":"$USER" "$TARGET_DIR"

# --- Create new Build target ---
docker compose -f "$COMPOSE_FILE" build --no-cache
docker compose -f "$COMPOSE_FILE" up -d


# --- (Optional) restart containers ---
if [ -f "$COMPOSE_FILE" ]; then
  echo "Restarting Docker services..."
  docker compose -f "$COMPOSE_FILE" down
  docker compose -f "$COMPOSE_FILE" up -d "$SERVER_CONTAINER"
fi

# --- Optionally run migrations ---
if [ "$RUN_MIGRATIONS" = true ]; then
  echo "Running EF Core migrations inside $SERVER_CONTAINER..."
  if docker exec "$SERVER_CONTAINER" dotnet ef database update; then
    echo "Migrations applied successfully."
  else
    echo "Migration failed or EF tools missing."
  fi
fi

echo "Deployment to $TARGET_DIR complete!"
