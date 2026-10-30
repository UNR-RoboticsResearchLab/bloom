// Outbound connections to the backend: TCP (direct, or through an HTTP proxy with CONNECT), then
// TLS with SNI, chain and hostname verification. Synchronous with timeouts -- the tunnel goes async
// only once the WebSocket is up.
#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/ssl.hpp>

#include "config.h"
#include "proxy.h"

namespace bloom_agent {

namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
using TlsStream = beast::ssl_stream<beast::tcp_stream>;

// Human-readable error without Boost's "[system:111 at /usr/include/...]" source-location suffix.
std::string error_text(const beast::error_code& ec);
std::string error_text(const std::exception& e);

// System CA store plus cfg.ca_file, peer verification on, TLS 1.2 minimum.
ssl::context make_tls_context(const AgentConfig& cfg);

// Plain TCP to host:port, or to the proxy followed by a CONNECT for host:port. Throws
// beast::system_error, or std::runtime_error when the proxy refuses the CONNECT.
void connect_tcp(beast::tcp_stream& stream, const std::string& host, const std::string& port,
                 const std::optional<ProxyConfig>& proxy, std::chrono::seconds timeout);

// connect_tcp + TLS handshake verifying the certificate chain and that it's issued for `url.host`.
void connect_tls(TlsStream& stream, const BackendUrl& url, const std::optional<ProxyConfig>& proxy,
                 std::chrono::seconds timeout);

struct HttpResult {
  unsigned status = 0;
  std::string body;
  std::string location;
};

// One HTTPS request over a fresh verified connection. Throws on network/TLS failure.
HttpResult https_request(ssl::context& tls, const AgentConfig& cfg, const std::string& method,
                         const std::string& target, const std::string& json_body = "");

}  // namespace bloom_agent
