// The OTA manager's view of the robot: how updates are launched, cancelled and observed. The real
// implementation runs `fleet-ota` as its own systemd unit.
//
// Why `systemd-run --unit=fleet-ota-<jobId> --collect` rather than a templated unit
// (fleet-ota@.service): the unit is created on demand with a name derived from the job id, so
// there is nothing to install or keep in sync on the robot besides the script, and `--setenv`
// carries per-job settings. Either way dpkg ends up in a unit of its own, not in
// bloom-agent.service's cgroup, so restarting or crashing the agent (KillMode=control-group)
// cannot kill an install halfway. Arguments are always passed as an array: no shell is involved.
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "config.h"
#include "ota_manifest.h"
#include "ota_state.h"

namespace bloom_agent::ota {

struct LaunchRequest {
  std::string job_id;
  Action action = Action::Stage;
  std::string version;
};

struct Outcome {
  bool ok = false;
  std::string message;
};

class IOtaSystem {
 public:
  virtual ~IOtaSystem() = default;
  // Starts the updater detached and returns as soon as systemd has accepted it.
  virtual Outcome launch(const LaunchRequest& request) = 0;
  // Asks the updater to stop at its next checkpoint. Never signals dpkg.
  virtual Outcome request_cancel(const std::string& job_id) = 0;
  virtual bool unit_active(const std::string& job_id) = 0;
  // Version of the metapackage, "" when it is not installed.
  virtual std::string installed_version() = 0;
  // The updater's state file, if it exists and parses.
  virtual std::optional<JobStatus> read_state() = 0;
};

struct CommandResult {
  int exit_code = -1;  // -1: could not run, or killed by a signal
  std::string output;  // stdout+stderr, truncated
  bool timed_out = false;
};
// fork/exec of an argument vector (no shell); the child gets stdio only. Killed after `timeout`.
CommandResult run_command(const std::vector<std::string>& argv, std::chrono::seconds timeout);

// The exact argument vector used to start an update. Pure, so tests can inspect it. `prefix_sudo`
// is true when the agent is not root.
std::vector<std::string> build_launch_argv(const OtaConfig& cfg, const LaunchRequest& request, bool prefix_sudo);

class SystemdOtaSystem : public IOtaSystem {
 public:
  SystemdOtaSystem(OtaConfig cfg, bool use_sudo) : cfg_(std::move(cfg)), use_sudo_(use_sudo) {}
  Outcome launch(const LaunchRequest& request) override;
  Outcome request_cancel(const std::string& job_id) override;
  bool unit_active(const std::string& job_id) override;
  std::string installed_version() override;
  std::optional<JobStatus> read_state() override;

 private:
  OtaConfig cfg_;
  bool use_sudo_;
};

}  // namespace bloom_agent::ota
