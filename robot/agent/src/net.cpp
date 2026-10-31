#include "net.h"

#include <openssl/err.h>
#include <openssl/ssl.h>

#include <boost/asio/connect.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/http.hpp>
#include <stdexcept>

namespace bloom_agent {

namespace http = beast::http;
using tcp = boost::asio::ip::tcp;

std::string error_text(const beast::error_code& ec) {
  if (ec.category() == boost::asio::error::get_ssl_category()) {
    // OpenSSL's reason string is more useful than asio's generic message.
    char buf[256];
    ERR_error_string_n(static_cast<unsigned long>(ec.value()), buf, sizeof(buf));
    return buf;
  }
  return ec.message();
}

std::string error_text(const std::exception& e) {
  if (const auto* se = dynamic_cast<const beast::system_error*>(&e)) return error_text(se->code());
  return e.what();
}

ssl::context make_tls_context(const AgentConfig& cfg) {
  ssl::context ctx(ssl::context::tls_client);
  ctx.set_options(ssl::context::default_workarounds | ssl::context::no_sslv2 | ssl::context::no_sslv3 |
                  ssl::context::no_tlsv1 | ssl::context::no_tlsv1_1);
  ctx.set_verify_mode(ssl::verify_peer);
  ctx.set_default_verify_paths();
  if (!cfg.ca_file.empty()) ctx.load_verify_file(cfg.ca_file);
  return ctx;
}

void connect_tcp(beast::tcp_stream& stream, const std::string& host, const std::string& port,
                 const std::optional<ProxyConfig>& proxy, std::chrono::seconds timeout) {
  tcp::resolver resolver(stream.get_executor());
  stream.expires_after(timeout);
  if (!proxy) {
    stream.connect(resolver.resolve(host, port));
    return;
  }

  stream.connect(resolver.resolve(proxy->host, proxy->port));
  const auto request = connect_request(host, port, *proxy);
  boost::asio::write(stream, boost::asio::buffer(request));

  // A CONNECT response has headers but no body, whatever its Content-Length says.
  beast::flat_buffer buffer;
  http::response_parser<http::empty_body> parser;
  parser.skip(true);
  http::read(stream, buffer, parser);
  const auto status = parser.get().result_int();
  if (status != 200)
    throw std::runtime_error("proxy " + proxy->host + ":" + proxy->port + " refused CONNECT with HTTP " +
                             std::to_string(status));
}

void connect_tls(TlsStream& stream, const BackendUrl& url, const std::optional<ProxyConfig>& proxy,
                 std::chrono::seconds timeout) {
  connect_tcp(beast::get_lowest_layer(stream), url.host, url.port, proxy, timeout);

  if (!SSL_set_tlsext_host_name(stream.native_handle(), url.host.c_str()))
    throw beast::system_error(beast::error_code(static_cast<int>(::ERR_get_error()), boost::asio::error::get_ssl_category()));
  stream.set_verify_callback(ssl::host_name_verification(url.host));
  beast::get_lowest_layer(stream).expires_after(timeout);
  stream.handshake(ssl::stream_base::client);
}

HttpResult https_request(ssl::context& tls, const AgentConfig& cfg, const std::string& method,
                         const std::string& target, const std::string& json_body) {
  boost::asio::io_context io;
  TlsStream stream(io, tls);
  connect_tls(stream, cfg.backend, proxy_for(cfg.backend.host), std::chrono::seconds(15));

  http::request<http::string_body> req{http::string_to_verb(method), cfg.backend.base_path + target, 11};
  req.set(http::field::host, cfg.backend.authority());
  req.set(http::field::user_agent, "bloom-robot-agent");
  if (!json_body.empty()) {
    req.set(http::field::content_type, "application/json");
    req.body() = json_body;
  }
  req.prepare_payload();
  http::write(stream, req);

  beast::flat_buffer buffer;
  http::response<http::string_body> res;
  http::read(stream, buffer, res);

  beast::error_code ignored;
  stream.shutdown(ignored);
  return {res.result_int(), res.body(), std::string(res[http::field::location])};
}

}  // namespace bloom_agent
