// `robot-agent doctor`: walks the connection path one layer at a time and says which layer breaks,
// for whoever is standing next to a robot on an unfamiliar network. Read-only: it never sends
// `hello`, so it doesn't disturb a running agent's tunnel.

#include "doctor.h"

#include <openssl/x509v3.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <cstdio>
#include <cstdlib>
#include <nlohmann/json.hpp>

#include "net.h"
#include "version.h"

namespace bloom_agent {

namespace http = beast::http;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace {

int failures = 0;

void pass(const std::string& check, const std::string& detail) {
  std::printf("  ✓ %-18s %s\n", check.c_str(), detail.c_str());
}

void fail(const std::string& check, const std::string& detail, const std::string& hint = "") {
  ++failures;
  std::printf("  ✗ %-18s %s\n", check.c_str(), detail.c_str());
  if (!hint.empty()) std::printf("    %-18s hint: %s\n", "", hint.c_str());
}

void note(const std::string& check, const std::string& detail) {
  std::printf("  - %-18s %s\n", check.c_str(), detail.c_str());
}

std::string x509_name(X509_NAME* name) {
  char buf[512];
  X509_NAME_oneline(name, buf, sizeof(buf));
  return buf;
}

std::string not_after(X509* cert) {
  const ASN1_TIME* t = X509_get0_notAfter(cert);
  BIO* bio = BIO_new(BIO_s_mem());
  ASN1_TIME_print(bio, t);
  char buf[128] = {};
  BIO_read(bio, buf, sizeof(buf) - 1);
  BIO_free(bio);
  return buf;
}

struct ParsedUrl {
  std::string scheme, host, port, target;
};

std::optional<ParsedUrl> parse_plain_url(const std::string& url) {
  const auto scheme_end = url.find("://");
  if (scheme_end == std::string::npos) return std::nullopt;
  ParsedUrl u{url.substr(0, scheme_end), "", "", "/"};
  auto rest = url.substr(scheme_end + 3);
  if (const auto slash = rest.find('/'); slash != std::string::npos) {
    u.target = rest.substr(slash);
    rest = rest.substr(0, slash);
  }
  if (const auto colon = rest.rfind(':'); colon != std::string::npos) {
    u.port = rest.substr(colon + 1);
    rest = rest.substr(0, colon);
  } else {
    u.port = u.scheme == "https" ? "443" : "80";
  }
  u.host = rest;
  return u;
}

// TLS handshake that records verification problems instead of aborting, so the certificate can be
// shown -- an intercepting middlebox's certificate is the most useful thing to print.
bool check_tls(const AgentConfig& cfg, const std::optional<ProxyConfig>& proxy) {
  asio::io_context io;
  ssl::context ctx = make_tls_context(cfg);
  TlsStream stream(io, ctx);

  int verify_error = X509_V_OK;
  stream.set_verify_callback([&](bool ok, ssl::verify_context& vctx) {
    if (!ok && verify_error == X509_V_OK) verify_error = X509_STORE_CTX_get_error(vctx.native_handle());
    return true;
  });
  try {
    connect_tcp(beast::get_lowest_layer(stream), cfg.backend.host, cfg.backend.port, proxy, std::chrono::seconds(10));
    SSL_set_tlsext_host_name(stream.native_handle(), cfg.backend.host.c_str());
    stream.handshake(ssl::stream_base::client);
  } catch (const std::exception& e) {
    fail("TLS handshake", error_text(e),
         "the connection was reset during TLS: a firewall may be blocking non-browser TLS, or the port isn't TLS");
    return false;
  }

  X509* cert = SSL_get1_peer_certificate(stream.native_handle());
  if (!cert) {
    fail("TLS certificate", "server sent no certificate");
    return false;
  }
  const auto issuer = x509_name(X509_get_issuer_name(cert));
  const auto subject = x509_name(X509_get_subject_name(cert));
  const bool host_ok = X509_check_host(cert, cfg.backend.host.c_str(), cfg.backend.host.size(), 0, nullptr) == 1;
  const auto expires = not_after(cert);
  X509_free(cert);

  const std::string intercept_hint =
      "this network appears to intercept TLS (common on campus/corporate Wi-Fi). Ask IT to exempt the backend "
      "host from inspection, or if policy requires inspection, add their CA certificate via ca_file in the config";

  bool ok = true;
  if (verify_error != X509_V_OK) {
    fail("TLS certificate", std::string("not trusted: ") + X509_verify_cert_error_string(verify_error), intercept_hint);
    ok = false;
  } else if (!host_ok) {
    fail("TLS certificate", "not issued for " + cfg.backend.host, intercept_hint);
    ok = false;
  } else {
    pass("TLS certificate", "trusted, valid for " + cfg.backend.host + ", expires " + expires);
  }

  if (!cfg.expected_issuers.empty()) {
    bool matched = false;
    for (const auto& expected : cfg.expected_issuers) matched |= issuer.find(expected) != std::string::npos;
    if (!matched) {
      fail("TLS issuer", "unexpected issuer " + issuer, intercept_hint);
      ok = false;
    } else {
      pass("TLS issuer", issuer);
    }
  } else {
    note("TLS issuer", issuer);
  }
  note("TLS subject", subject);

  beast::error_code ignored;
  stream.shutdown(ignored);
  return ok;
}

// Classic captive-portal detection: a plain-HTTP URL that answers 204 on an open network. A portal
// answers with its login page or a redirect instead.
void check_captive_portal(const AgentConfig& cfg) {
  const auto url = parse_plain_url(cfg.captive_check_url);
  if (!url || url->scheme != "http") {
    note("Captive portal", "skipped (captive_check_url must be http://)");
    return;
  }
  try {
    asio::io_context io;
    beast::tcp_stream stream(io);
    tcp::resolver resolver(io);
    stream.expires_after(std::chrono::seconds(10));
    stream.connect(resolver.resolve(url->host, url->port));
    http::request<http::empty_body> req{http::verb::get, url->target, 11};
    req.set(http::field::host, url->host);
    req.set(http::field::user_agent, "bloom-robot-agent");
    http::write(stream, req);
    beast::flat_buffer buffer;
    http::response<http::string_body> res;
    http::read(stream, buffer, res);

    if (res.result_int() == 204)
      pass("Captive portal", "none detected");
    else
      fail("Captive portal", "probe got HTTP " + std::to_string(res.result_int()) +
                                 (res[http::field::location].empty() ? "" : " -> " + std::string(res[http::field::location])),
           "this network wants a browser login or device registration (often by MAC address) before it allows traffic");
  } catch (const std::exception& e) {
    note("Captive portal", std::string("probe failed (") + error_text(e) + "); outbound port 80 may be blocked, which is fine");
  }
}

}  // namespace

int run_doctor(const std::string& config_path) {
  std::printf("robot-agent %s (%s) doctor\n\n", kVersion, kCommit);
  failures = 0;

  AgentConfig cfg;
  try {
    cfg = load_config(config_path, /*require_credentials=*/false);
    pass("Config", config_path + ": backend " + cfg.backend_url);
  } catch (const std::exception& e) {
    fail("Config", error_text(e), "see /etc/bloom-agent/agent.json.example");
    return 1;
  }
  const bool have_credentials = !cfg.robot_id.empty() && !cfg.secret.empty();
  if (have_credentials)
    pass("Credentials", "robot " + cfg.robot_id + ", secret present");
  else
    fail("Credentials", cfg.robot_id.empty() ? "robot_id missing" : "secret missing",
         "an admin issues them with POST /api/robot/{id}/credentials; install.sh writes them");

  // ── Proxy ──
  std::optional<ProxyConfig> proxy;
  bool proxy_broken = false;
  for (const char* name : {"HTTPS_PROXY", "https_proxy", "ALL_PROXY", "all_proxy"}) {
    if (const char* v = std::getenv(name); v && *v && !parse_proxy_url(v)) {
      fail("Proxy", std::string(name) + "=" + v + " is not an http:// proxy URL", "only HTTP CONNECT proxies are supported");
      proxy_broken = true;
    }
  }
  if (!proxy_broken) {
    proxy = proxy_for(cfg.backend.host);
    if (proxy)
      pass("Proxy", "via " + proxy->host + ":" + proxy->port + " (from " + proxy->source + ")");
    else
      pass("Proxy", "direct connection (no HTTPS_PROXY, or NO_PROXY matches)");
  }

  // ── DNS ──
  const std::string dns_host = proxy ? proxy->host : cfg.backend.host;
  const std::string dns_port = proxy ? proxy->port : cfg.backend.port;
  try {
    asio::io_context io;
    tcp::resolver resolver(io);
    auto results = resolver.resolve(dns_host, dns_port);
    std::string addrs;
    for (const auto& r : results) addrs += (addrs.empty() ? "" : ", ") + r.endpoint().address().to_string();
    pass("DNS", dns_host + " -> " + addrs);
  } catch (const std::exception& e) {
    fail("DNS", dns_host + ": " + error_text(e), "check /etc/resolv.conf / `resolvectl status`; the network's DNS may block it");
    return 1;
  }

  // ── TCP (and proxy CONNECT) ──
  try {
    asio::io_context io;
    beast::tcp_stream stream(io);
    connect_tcp(stream, cfg.backend.host, cfg.backend.port, proxy, std::chrono::seconds(10));
    pass(proxy ? "Proxy CONNECT" : "TCP " + cfg.backend.port, "connected to " + cfg.backend.authority());
  } catch (const std::exception& e) {
    fail(proxy ? "Proxy CONNECT" : "TCP " + cfg.backend.port, error_text(e),
         proxy ? "the proxy refused or the backend is unreachable from it"
               : "outbound " + cfg.backend.port + " is blocked or the backend is down; this network may require a proxy");
    return 1;
  }

  // ── TLS ──
  const bool tls_ok = check_tls(cfg, proxy);
  check_captive_portal(cfg);
  if (!tls_ok) {
    std::printf("\n%d check(s) failed; skipping backend checks until TLS is trusted.\n", failures);
    return 1;
  }

  // ── Backend health over verified TLS ──
  ssl::context tls = make_tls_context(cfg);
  try {
    const auto res = https_request(tls, cfg, "GET", "/api/agent/health");
    const auto json = nlohmann::json::parse(res.body, nullptr, false);
    if (res.status == 200 && json.is_object() && json.value("service", "") == "bloom")
      pass("Backend", "healthy, version " + json.value("version", "?"));
    else
      fail("Backend", "unexpected answer: HTTP " + std::to_string(res.status) + (res.location.empty() ? "" : " -> " + res.location),
           "something other than the Bloom backend answered: a portal, proxy error page, or wrong backend_url");
  } catch (const std::exception& e) {
    fail("Backend", error_text(e));
  }

  if (!have_credentials) {
    std::printf("\n%d check(s) failed.\n", failures);
    return 1;
  }

  // ── Login ──
  std::string token;
  try {
    const auto body = nlohmann::json{{"robotId", cfg.robot_id}, {"secret", cfg.secret}}.dump();
    const auto res = https_request(tls, cfg, "POST", "/api/robot/login", body);
    const auto json = nlohmann::json::parse(res.body, nullptr, false);
    if (res.status == 200 && json.is_object() && json.contains("token")) {
      token = json["token"].get<std::string>();
      pass("Login", "credentials accepted");
    } else if (res.status == 401) {
      fail("Login", "credentials rejected (HTTP 401)", "the robot was revoked or the secret is wrong; ask an admin to re-issue it");
    } else {
      fail("Login", "HTTP " + std::to_string(res.status));
    }
  } catch (const std::exception& e) {
    fail("Login", error_text(e));
  }

  // ── WebSocket upgrade ──
  if (!token.empty()) {
    try {
      asio::io_context io;
      websocket::stream<TlsStream> ws(io, tls);
      connect_tls(ws.next_layer(), cfg.backend, proxy, std::chrono::seconds(10));
      ws.set_option(websocket::stream_base::decorator(
          [&token](websocket::request_type& req) { req.set(http::field::authorization, "Bearer " + token); }));
      websocket::response_type res;
      beast::error_code ec;
      ws.handshake(res, cfg.backend.authority(), cfg.backend.base_path + "/ws/robot-agent", ec);
      if (!ec) {
        pass("WebSocket", "upgrade accepted on /ws/robot-agent");
        ws.close(websocket::close_code::normal, ec);
      } else if (res.result_int() == 404) {
        fail("WebSocket", "HTTP 404", "the backend has RemoteShell:Enabled=false");
      } else if (res.result_int() != 0) {
        fail("WebSocket", "upgrade refused: HTTP " + std::to_string(res.result_int()));
      } else {
        fail("WebSocket", error_text(ec), "a proxy or firewall on this network may strip WebSocket upgrades");
      }
    } catch (const std::exception& e) {
      fail("WebSocket", error_text(e));
    }
  }

  std::printf("\n");
  if (failures == 0)
    std::printf("All checks passed.\n");
  else
    std::printf("%d check(s) failed.\n", failures);
  return failures == 0 ? 0 : 1;
}

}  // namespace bloom_agent
