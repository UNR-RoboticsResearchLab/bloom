#!/usr/bin/env bash
# Builds the robot .debs and the metapackage inside the build container.
#   robot/packaging/docker/build.sh [--platform linux/amd64|linux/arm64] [--semver X] [--meta-name NAME]
# Output: robot/packaging/dist/ (git-ignored). The repo is mounted read-only; nothing on the host
# is installed or modified.
set -euo pipefail

PLATFORM="linux/$(dpkg --print-architecture)"
SEMVER=""
META_NAME="bloom-robot-core"
while [[ $# -gt 0 ]]; do
  case $1 in
    --platform) PLATFORM=$2; shift 2 ;;
    --semver) SEMVER=$2; shift 2 ;;
    --meta-name) META_NAME=$2; shift 2 ;;
    *) echo "unknown option: $1" >&2; exit 64 ;;
  esac
done

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
[[ -n $SEMVER ]] || SEMVER="$(cd "$REPO" && nbgv get-version -v SemVer2)"
IMAGE="bloom-deb-build:${PLATFORM#linux/}"
OUT="$REPO/robot/packaging/dist/${PLATFORM#linux/}"
mkdir -p "$OUT"

docker build --platform "$PLATFORM" -t "$IMAGE" "$HERE"
docker run --rm --platform "$PLATFORM" \
  -v "$REPO:/src:ro" -v "$OUT:/out" \
  "$IMAGE" bash -c '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash   # not safe under set -u
    set -u
    export PYTHONPATH=/src/robot/packaging:${PYTHONPATH:-}
    cd /src/robot/packaging
    python3 -m bloom_packaging build --src ../src --out /out --semver "'"$SEMVER"'" --execute --install-built
    python3 -m bloom_packaging meta --name "'"$META_NAME"'" --semver "'"$SEMVER"'" --from-dir /out --out /out \
      --extra-depends "libportaudio2, alsa-utils, pulseaudio-utils" --env-file /etc/bloom/robot.env
  '
echo "built into $OUT"; ls -la "$OUT"
