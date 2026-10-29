// HTTPS proxy support. Campus and corporate networks often only allow egress through an HTTP
// proxy, configured system-wide via HTTPS_PROXY/https_proxy and NO_PROXY (/etc/environment,
// which the systemd unit loads). Beast has no proxy support, so the agent tunnels through it with
// HTTP CONNECT itself, then does TLS end-to-end with the backend over that tunnel.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace bloom_agent {

struct ProxyConfig {
  std::string host;
  std::string port;
  std::string authorization;  // full Proxy-Authorization value ("Basic ..."), or empty
  std::string source;         // which variable it came from, for doctor output
};

using EnvLookup = std::function<const char*(const char*)>;
const char* system_env(const char* name);

// Proxy to use for connecting to target_host, or nullopt for a direct connection.
std::optional<ProxyConfig> proxy_for(std::string_view target_host, const EnvLookup& env = system_env);

// NO_PROXY: comma-separated hostnames/domain suffixes ("example.edu", ".example.edu") or "*".
// IP ranges and ports in entries aren't supported; they never match.
bool no_proxy_matches(std::string_view host, std::string_view no_proxy);

std::optional<ProxyConfig> parse_proxy_url(std::string_view url);

std::string connect_request(std::string_view host, std::string_view port, const ProxyConfig& proxy);

}  // namespace bloom_agent
