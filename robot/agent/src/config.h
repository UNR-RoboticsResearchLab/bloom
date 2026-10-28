// Agent configuration. Nothing secret or deployment-specific is compiled in.
//
// File: /etc/bloom-agent/agent.json (or --config / BLOOM_AGENT_CONFIG):
//   { "backend_url": "https://bloom.example.edu", "robot_id": "<guid>",
//     "ca_file": "", "expected_issuers": [], "captive_check_url": "...", "shell": "" }
// Env overrides: BLOOM_AGENT_BACKEND_URL, BLOOM_ROBOT_ID.
// Secret, first found: $CREDENTIALS_DIRECTORY/secret (systemd LoadCredential),
//   /etc/bloom-agent/secret, BLOOM_ROBOT_SECRET (dev only).
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace bloom_agent {

struct BackendUrl {
  std::string host;
  std::string port = "443";
  std::string base_path;  // no trailing slash; "" for the root

  // Host header value: host, plus :port when it isn't 443.
  std::string authority() const { return port == "443" ? host : host + ":" + port; }

  // https only -- the tunnel must be TLS end to end.
  static std::optional<BackendUrl> parse(const std::string& url);
};

struct AgentConfig {
  BackendUrl backend;
  std::string backend_url;
  std::string robot_id;
  std::string secret;
  std::string ca_file;                    // extra CA bundle (private CA, dev cert); system store is always used too
  std::vector<std::string> expected_issuers;  // doctor: substrings expected in the server cert's issuer
  std::string captive_check_url = "http://connectivity-check.ubuntu.com/";
  std::string shell;                      // default: the agent user's login shell
  // Run (via sudo, unless root) with the new secret on stdin when the backend pushes one. Writes
  // the agent's secret file and bloom_node's .env.
  std::string apply_secret_command = "/opt/bloom-agent/current/provision.sh";
  std::string config_path;
};

constexpr const char* kDefaultConfigPath = "/etc/bloom-agent/agent.json";
constexpr const char* kDefaultSecretPath = "/etc/bloom-agent/secret";

// True when the secret comes from systemd's $CREDENTIALS_DIRECTORY, which the agent can't replace.
bool secret_is_externally_managed();

// Reads the secret from the first source that has one (see top of file); empty if none. Called
// before every login, so installing a rotated secret doesn't need an agent restart.
std::string read_secret();

// Throws std::runtime_error with a message fit for printing. `doctor` passes
// require_credentials=false so it can still run the network checks on a half-configured robot.
AgentConfig load_config(const std::string& path, bool require_credentials = true);

}  // namespace bloom_agent
