#!/usr/bin/env bats
# Drives deploy/fleet-ota against stubs (tests/fleet-ota/stub): no real apt, dpkg or systemd.
#   npm install bats && npx bats robot/agent/tests/fleet-ota

SCRIPT="$BATS_TEST_DIRNAME/../../deploy/fleet-ota"
STUB="$BATS_TEST_DIRNAME/stub"

setup() {
  export STUB_LOG="$BATS_TEST_TMPDIR/calls.log"
  export STUB_STATE_DIR="$BATS_TEST_TMPDIR/state"
  export STUB_INSTALLED="$BATS_TEST_TMPDIR/installed"
  : >"$STUB_LOG"
  printf '1.0.0' >"$STUB_INSTALLED"

  local bin="$BATS_TEST_TMPDIR/bin"
  mkdir -p "$bin"
  for n in apt-get dpkg dpkg-query systemctl robot-is-idle robot-healthcheck; do ln -s "$STUB" "$bin/$n"; done

  export FLEET_OTA_STATE_DIR="$STUB_STATE_DIR"
  export FLEET_OTA_JOB_ID="job-1"
  export FLEET_OTA_APT_SOURCES="/etc/apt/sources.list.d/test.list"
  export FLEET_OTA_APT_GET="$bin/apt-get" FLEET_OTA_DPKG="$bin/dpkg" FLEET_OTA_DPKG_QUERY="$bin/dpkg-query"
  export FLEET_OTA_SYSTEMCTL="$bin/systemctl"
  export FLEET_OTA_IDLE_CMD="$bin/robot-is-idle" FLEET_OTA_HEALTH_CMD="$bin/robot-healthcheck"
  export FLEET_OTA_HEALTH_TIMEOUT=5
}

state() { jq -r ".$1" "$STUB_STATE_DIR/state.json"; }
calls() { cat "$STUB_LOG"; }

@test "stage: updates from our source only, downloads the exact version, installs nothing" {
  run "$SCRIPT" stage 1.4.2-1jazzy
  [ "$status" -eq 0 ]
  [ "$(state state)" = staged ]
  [ "$(state version)" = 1.4.2-1jazzy ]
  [ "$(state action)" = stage ]
  grep -q '^dpkg --configure -a$' "$STUB_LOG"
  grep -q 'Dir::Etc::sourcelist=/etc/apt/sources.list.d/test.list' <(grep '^apt-get.* update' "$STUB_LOG")
  grep -q 'Dir::Etc::sourceparts=-' <(grep '^apt-get.* update' "$STUB_LOG")
  grep -q 'install --download-only --allow-downgrades bloom-robot-core=1.4.2-1jazzy' "$STUB_LOG"
  ! grep -q '^systemctl' "$STUB_LOG"
  [ "$(cat "$STUB_INSTALLED")" = 1.0.0 ]
}

@test "dpkg --configure -a runs before any apt-get" {
  run "$SCRIPT" stage 1.4.2
  [ "$status" -eq 0 ]
  [ "$(sed -n 1p "$STUB_LOG")" = "dpkg --configure -a" ]
}

@test "authentication is never relaxed and every apt call is restricted" {
  run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 0 ]
  [ "$(grep -c '^apt-get' "$STUB_LOG")" -ge 1 ]
  ! grep -q -- '--allow-unauthenticated' "$SCRIPT"
  ! grep -q -- 'allow-unauthenticated' "$STUB_LOG"
  while read -r line; do
    [[ $line == *'AllowUnauthenticated=false'* ]]
    [[ $line == *'Dir::Etc::sourcelist='* ]]
  done < <(grep '^apt-get' "$STUB_LOG")
}

@test "stage: a failed download fails the job" {
  STUB_DOWNLOAD_FAIL=1 run "$SCRIPT" stage 1.4.2
  [ "$status" -eq 1 ]
  [ "$(state state)" = failed ]
}

@test "stage: a failed update fails the job before downloading" {
  STUB_UPDATE_FAIL=1 run "$SCRIPT" stage 1.4.2
  [ "$status" -eq 1 ]
  [ "$(state state)" = failed ]
  ! grep -q -- '--download-only' "$STUB_LOG"
}

@test "stage: a broken package database stops the job" {
  STUB_DPKG_FAIL=1 run "$SCRIPT" stage 1.4.2
  [ "$status" -eq 1 ]
  [ "$(state state)" = failed ]
  ! grep -q '^apt-get' "$STUB_LOG"
}

@test "apply: installs, restarts the ROS service, health-checks, succeeds" {
  run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 0 ]
  [ "$(state state)" = success ]
  [ "$(state previousVersion)" = 1.0.0 ]
  [ "$(cat "$STUB_INSTALLED")" = 2.0.0 ]
  grep -q 'install --allow-downgrades bloom-robot-core=2.0.0' "$STUB_LOG"
  grep -q '^systemctl restart bloom.service$' "$STUB_LOG"
  grep -q '^robot-healthcheck' "$STUB_LOG"
}

@test "apply: a busy robot defers and is left untouched" {
  STUB_BUSY=1 run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 0 ]
  [ "$(state state)" = deferred ]
  [ "$(cat "$STUB_INSTALLED")" = 1.0.0 ]
  ! grep -q 'install' "$STUB_LOG"
  ! grep -q '^systemctl' "$STUB_LOG"
}

@test "apply: an unhealthy new version is rolled back to the previous one" {
  STUB_UNHEALTHY_VERSION=2.0.0 run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 1 ]
  [ "$(state state)" = rolled_back ]
  [ "$(cat "$STUB_INSTALLED")" = 1.0.0 ]
  grep -q 'install --allow-downgrades bloom-robot-core=1.0.0' "$STUB_LOG"
  [ "$(grep -c '^systemctl restart' "$STUB_LOG")" -eq 2 ]
}

@test "apply: a failed install is rolled back" {
  STUB_INSTALL_FAIL_VERSION=2.0.0 run "$SCRIPT" apply 2.0.0
  [ "$(state state)" = rolled_back ]
  [ "$(cat "$STUB_INSTALLED")" = 1.0.0 ]
}

@test "apply: a failed service restart is rolled back" {
  STUB_RESTART_FAIL=2.0.0 run "$SCRIPT" apply 2.0.0
  [ "$(state state)" = rolled_back ]
  [ "$(cat "$STUB_INSTALLED")" = 1.0.0 ]
}

@test "apply: when the rollback fails too the job is failed, not rolled_back" {
  STUB_UNHEALTHY_VERSION=2.0.0 STUB_INSTALL_FAIL_VERSION=1.0.0 run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 1 ]
  [ "$(state state)" = failed ]
}

@test "apply: with no previous version a failure cannot roll back" {
  rm "$STUB_INSTALLED"
  STUB_INSTALL_FAIL_VERSION=2.0.0 run "$SCRIPT" apply 2.0.0
  [ "$(state state)" = failed ]
}

@test "apply: already at the target version is a no-op success" {
  run "$SCRIPT" apply 1.0.0
  [ "$status" -eq 0 ]
  [ "$(state state)" = success ]
  ! grep -q 'install' "$STUB_LOG"
}

@test "apply: downgrades are allowed" {
  printf '3.0.0' >"$STUB_INSTALLED"
  run "$SCRIPT" apply 2.0.0
  [ "$(state state)" = success ]
  grep -q -- '--allow-downgrades' "$STUB_LOG"
}

@test "cancel before the point of no return stops a stage" {
  "$SCRIPT" cancel job-1
  run "$SCRIPT" stage 1.4.2
  [ "$status" -eq 0 ]
  [ "$(state state)" = cancelled ]
  ! grep -q -- '--download-only' "$STUB_LOG"
}

@test "cancel before an apply installs stops it" {
  "$SCRIPT" cancel job-1
  run "$SCRIPT" apply 2.0.0
  [ "$(state state)" = cancelled ]
  [ "$(cat "$STUB_INSTALLED")" = 1.0.0 ]
}

@test "cancel during an install is ignored: dpkg is never interrupted" {
  STUB_CANCEL_DURING_INSTALL=1 run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 0 ]
  [ "$(state state)" = success ]
  [ "$(cat "$STUB_INSTALLED")" = 2.0.0 ]
}

@test "a second concurrent update is refused without touching state.json" {
  mkdir -p "$STUB_STATE_DIR"
  echo '{"jobId":"other","state":"applying"}' >"$STUB_STATE_DIR/state.json"
  exec 8>"$STUB_STATE_DIR/lock"
  flock -n 8
  run "$SCRIPT" apply 2.0.0
  [ "$status" -ne 0 ]
  [ "$(state jobId)" = other ]
  ! grep -q '^apt-get' "$STUB_LOG"
  exec 8>&-
}

@test "bad versions are refused before anything runs" {
  for v in '' '1.0;reboot' '$(id)' '-1' 'v1' '1 2' '1:2'; do
    run "$SCRIPT" apply "$v"
    [ "$status" -ne 0 ]
  done
  [ ! -s "$STUB_LOG" ]
}

@test "bad job ids and unknown commands are refused" {
  FLEET_OTA_JOB_ID='a b' run "$SCRIPT" stage 1.0.0
  [ "$status" -ne 0 ]
  FLEET_OTA_JOB_ID='' run "$SCRIPT" stage 1.0.0
  [ "$status" -ne 0 ]
  run "$SCRIPT" cancel '../x'
  [ "$status" -ne 0 ]
  run "$SCRIPT" reboot
  [ "$status" -ne 0 ]
  [ ! -s "$STUB_LOG" ]
}

@test "state.json is valid JSON with every field the agent reads" {
  run "$SCRIPT" apply 2.0.0
  jq -e '.jobId and .action and .version and .state and (.percent|type=="number") and .detail and .updatedAt' "$STUB_STATE_DIR/state.json"
}

@test "a missing idle check fails the job" {
  # A stub that kills the script mid-apply with set -e semantics: dpkg-query printing then the idle check missing.
  FLEET_OTA_IDLE_CMD=/nonexistent run "$SCRIPT" apply 2.0.0
  [ "$status" -eq 1 ]
  [ "$(state state)" = failed ]
}
