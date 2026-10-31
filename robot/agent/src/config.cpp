#include "config.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "protocol.h"

namespace bloom_agent {
namespace {
std::optional<std::string> read_trimmed(const std::string& path) {
  std::ifstream in(path);
  if (!in) return std::nullopt;
  std::stringstream ss;
  ss << in.rdbuf();
  std::string s = ss.str();
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
  return s;
}

std::string env_or(const char* name, const std::string& fallback) {
  const char* v = std::getenv(name);
  return v && *v ? std::string(v) : fallback;
}
}  // namespace

std::optional<BackendUrl> BackendUrl::parse(const std::string& url) {
  constexpr std::string_view scheme = "https://";
  if (url.compare(0, scheme.size(), scheme) != 0) return std::nullopt;
  std::string rest = url.substr(scheme.size());

  BackendUrl out;
  const auto slash = rest.find('/');
  if (slash != std::string::npos) {
    out.base_path = rest.substr(slash);
    while (!out.base_path.empty() && out.base_path.back() == '/') out.base_path.pop_back();
    rest = rest.substr(0, slash);
  }
  const auto colon = rest.rfind(':');
  if (colon != std::string::npos) {
    out.port = rest.substr(colon + 1);
    rest = rest.substr(0, colon);
    if (out.port.empty() || out.port.find_first_not_of("0123456789") != std::string::npos) return std::nullopt;
  }
  if (rest.empty()) return std::nullopt;
  out.host = rest;
  return out;
}

bool secret_is_externally_managed() {
  const char* dir = std::getenv("CREDENTIALS_DIRECTORY");
  if (!dir) return false;
  auto s = read_trimmed(std::string(dir) + "/secret");
  return s && !s->empty();
}

std::string read_secret() {
  if (const char* dir = std::getenv("CREDENTIALS_DIRECTORY"))
    if (auto s = read_trimmed(std::string(dir) + "/secret"); s && !s->empty()) return *s;
  // BLOOM_AGENT_SECRET_FILE moves the file (dev/test only), e.g. to run the agent unprivileged.
  if (auto s = read_trimmed(env_or("BLOOM_AGENT_SECRET_FILE", kDefaultSecretPath)); s && !s->empty()) return *s;
  return env_or("BLOOM_ROBOT_SECRET", "");
}

AgentConfig load_config(const std::string& path, bool require_credentials) {
  AgentConfig cfg;
  cfg.config_path = path;

  nlohmann::json file = nlohmann::json::object();
  if (auto text = read_trimmed(path)) {
    try {
      file = nlohmann::json::parse(*text);
    } catch (const nlohmann::json::exception& e) {
      throw std::runtime_error("invalid JSON in " + path + ": " + e.what());
    }
  } else if (!std::getenv("BLOOM_AGENT_BACKEND_URL")) {
    throw std::runtime_error("cannot read config file " + path);
  }

  auto str = [&](const char* key) { return file.contains(key) && file[key].is_string() ? file[key].get<std::string>() : std::string(); };
  cfg.backend_url = env_or("BLOOM_AGENT_BACKEND_URL", str("backend_url"));
  cfg.robot_id = env_or("BLOOM_ROBOT_ID", str("robot_id"));
  cfg.ca_file = str("ca_file");
  cfg.shell = str("shell");
  if (auto v = str("apply_secret_command"); !v.empty()) cfg.apply_secret_command = v;
  if (auto v = str("captive_check_url"); !v.empty()) cfg.captive_check_url = v;
  if (file.contains("expected_issuers") && file["expected_issuers"].is_array())
    for (const auto& i : file["expected_issuers"])
      if (i.is_string()) cfg.expected_issuers.push_back(i.get<std::string>());

  if (file.contains("ota") && file["ota"].is_object()) {
    const auto& o = file["ota"];
    auto ostr = [&](const char* key, std::string& out) {
      if (o.contains(key) && o[key].is_string() && !o[key].get<std::string>().empty()) out = o[key].get<std::string>();
    };
    if (o.contains("enabled") && o["enabled"].is_boolean()) cfg.ota.enabled = o["enabled"].get<bool>();
    ostr("apt_sources_path", cfg.ota.apt_sources_path);
    ostr("metapackage", cfg.ota.metapackage);
    ostr("signing_key_path", cfg.ota.signing_key_path);
    ostr("idle_command", cfg.ota.idle_command);
    ostr("healthcheck_command", cfg.ota.healthcheck_command);
    ostr("ros_service", cfg.ota.ros_service);
    ostr("updater_path", cfg.ota.updater_path);
    ostr("state_dir", cfg.ota.state_dir);
    ostr("job_store_path", cfg.ota.job_store_path);
    if (o.contains("healthcheck_timeout_seconds") && o["healthcheck_timeout_seconds"].is_number_integer())
      cfg.ota.healthcheck_timeout_seconds = o["healthcheck_timeout_seconds"].get<int>();
    if (o.contains("allowed_suites") && o["allowed_suites"].is_array())
      for (const auto& s : o["allowed_suites"])
        if (s.is_string()) cfg.ota.allowed_suites.push_back(s.get<std::string>());
    if (cfg.ota.enabled && (cfg.ota.healthcheck_timeout_seconds < 1 || cfg.ota.healthcheck_timeout_seconds > 3600))
      throw std::runtime_error("ota.healthcheck_timeout_seconds must be between 1 and 3600");
  }

  auto backend = BackendUrl::parse(cfg.backend_url);
  if (!backend)
    throw std::runtime_error("backend_url must be an https:// URL (got \"" + cfg.backend_url + "\")");
  cfg.backend = *backend;

  cfg.secret = read_secret();
  if (!require_credentials) return cfg;

  if (cfg.robot_id.empty()) throw std::runtime_error("robot_id is not configured");
  if (!parse_uuid(cfg.robot_id)) throw std::runtime_error("robot_id is not a GUID: " + cfg.robot_id);

  if (cfg.secret.empty())
    throw std::runtime_error(std::string("no robot secret: expected $CREDENTIALS_DIRECTORY/secret, ") + kDefaultSecretPath +
                             ", or BLOOM_ROBOT_SECRET");
  return cfg;
}

}  // namespace bloom_agent
