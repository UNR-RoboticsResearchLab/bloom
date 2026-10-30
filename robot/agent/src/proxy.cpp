#include "proxy.h"

#include "base64.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace bloom_agent {
namespace {
std::string lower(std::string_view s) {
  std::string out(s);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::tolower(c); });
  return out;
}

std::string_view trim(std::string_view s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
  return s;
}

std::string percent_decode(std::string_view s) {
  std::string out;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '%' && i + 2 < s.size() && std::isxdigit(static_cast<unsigned char>(s[i + 1])) &&
        std::isxdigit(static_cast<unsigned char>(s[i + 2]))) {
      out.push_back(static_cast<char>(std::stoi(std::string(s.substr(i + 1, 2)), nullptr, 16)));
      i += 2;
    } else {
      out.push_back(s[i]);
    }
  }
  return out;
}
}  // namespace

const char* system_env(const char* name) { return std::getenv(name); }

std::optional<ProxyConfig> parse_proxy_url(std::string_view url) {
  url = trim(url);
  if (url.empty()) return std::nullopt;

  const auto scheme_end = url.find("://");
  if (scheme_end != std::string_view::npos) {
    // CONNECT is spoken over plain HTTP to the proxy; https:// and socks proxies aren't supported.
    if (lower(url.substr(0, scheme_end)) != "http") return std::nullopt;
    url.remove_prefix(scheme_end + 3);
  }
  const auto path = url.find('/');
  if (path != std::string_view::npos) url = url.substr(0, path);

  ProxyConfig proxy;
  const auto at = url.rfind('@');
  if (at != std::string_view::npos) {
    proxy.authorization = "Basic " + base64_encode(percent_decode(url.substr(0, at)));
    url.remove_prefix(at + 1);
  }

  const auto colon = url.rfind(':');
  if (colon != std::string_view::npos && url.find(']') == std::string_view::npos) {
    proxy.host = std::string(url.substr(0, colon));
    proxy.port = std::string(url.substr(colon + 1));
  } else {
    proxy.host = std::string(url);
    proxy.port = "80";
  }
  if (proxy.host.empty() || proxy.port.empty()) return std::nullopt;
  return proxy;
}

bool no_proxy_matches(std::string_view host, std::string_view no_proxy) {
  const std::string h = lower(host);
  size_t start = 0;
  while (start <= no_proxy.size()) {
    auto comma = no_proxy.find(',', start);
    if (comma == std::string_view::npos) comma = no_proxy.size();
    std::string entry = lower(trim(no_proxy.substr(start, comma - start)));
    start = comma + 1;

    if (entry.empty()) continue;
    if (entry == "*") return true;
    if (entry.front() == '.') entry.erase(0, 1);
    if (h == entry) return true;
    if (h.size() > entry.size() && h.compare(h.size() - entry.size(), entry.size(), entry) == 0 &&
        h[h.size() - entry.size() - 1] == '.')
      return true;
  }
  return false;
}

std::optional<ProxyConfig> proxy_for(std::string_view target_host, const EnvLookup& env) {
  for (const char* name : {"NO_PROXY", "no_proxy"}) {
    if (const char* v = env(name); v && no_proxy_matches(target_host, v)) return std::nullopt;
  }
  for (const char* name : {"HTTPS_PROXY", "https_proxy", "ALL_PROXY", "all_proxy"}) {
    if (const char* v = env(name); v && *v) {
      auto proxy = parse_proxy_url(v);
      if (proxy) proxy->source = name;
      return proxy;
    }
  }
  return std::nullopt;
}

std::string connect_request(std::string_view host, std::string_view port, const ProxyConfig& proxy) {
  std::string authority = std::string(host) + ":" + std::string(port);
  std::string req = "CONNECT " + authority + " HTTP/1.1\r\nHost: " + authority + "\r\n";
  if (!proxy.authorization.empty()) req += "Proxy-Authorization: " + proxy.authorization + "\r\n";
  req += "User-Agent: bloom-robot-agent\r\n\r\n";
  return req;
}

}  // namespace bloom_agent
