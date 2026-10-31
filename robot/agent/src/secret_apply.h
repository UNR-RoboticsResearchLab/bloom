// Applies a secret pushed by the backend by running the provisioning script with the secret on
// its stdin. Never on the command line (visible in `ps`) and never logged.
#pragma once

#include <chrono>
#include <functional>
#include <string>

namespace bloom_agent {

struct ApplyResult {
  bool ok = false;
  std::string message;  // last line the script printed (or why it couldn't run), fit for an admin to read
};

struct ApplyOptions {
  std::string command;        // e.g. /opt/bloom-agent/current/provision.sh
  bool use_sudo = true;       // the agent runs as bloom-ops; the script needs root to write /etc and .env
  std::chrono::seconds timeout{30};
};

using SecretApplier = std::function<ApplyResult(const std::string& secret)>;

// Runs `[sudo -n] <command> --secret-stdin --no-restart-agent`. Blocks until it exits or times out
// (then it is killed). The agent's own service is deliberately not restarted by the script: the
// backend drops the tunnel next and the agent reconnects with the new secret by itself.
ApplyResult apply_secret(const ApplyOptions& options, const std::string& secret);

}  // namespace bloom_agent
