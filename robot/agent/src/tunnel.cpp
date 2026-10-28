#include "tunnel.h"

#include <poll.h>
#include <unistd.h>

#include <array>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <cerrno>
#include <deque>
#include <map>
#include <nlohmann/json.hpp>

#include "liveness.h"
#include "log.h"
#include "protocol.h"
#include "sessions.h"
#include "version.h"

namespace bloom_agent {

namespace http = beast::http;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using clock = std::chrono::steady_clock;

namespace {

constexpr auto kPingInterval = std::chrono::seconds(25);
constexpr auto kDeadAfter = std::chrono::seconds(60);
// Stop reading shell output while this much is waiting to go out (e.g. `cat` of a huge file over a
// slow link), so memory stays bounded; resume below kResumeBytes.
constexpr std::size_t kPauseBytes = 1 << 20;
constexpr std::size_t kResumeBytes = 256 << 10;

// Keystrokes into the PTY. The master is non-blocking; input is small, so wait briefly when the
// kernel buffer is full (a big paste) rather than queueing.
void write_all(int fd, std::string_view data) {
  for (int stalls = 0; !data.empty() && stalls < 20;) {
    const ssize_t n = ::write(fd, data.data(), data.size());
    if (n > 0) {
      data.remove_prefix(static_cast<size_t>(n));
      stalls = 0;
    } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      pollfd p{fd, POLLOUT, 0};
      ::poll(&p, 1, 100);
      ++stalls;
    } else {
      return;  // EIO: the shell is gone; the reader will report it
    }
  }
}

// One established tunnel. Everything runs on one thread inside io.run(): WebSocket reads and
// writes, every PTY's output, and the keepalive timer.
class TunnelSession {
 public:
  using Ws = websocket::stream<TlsStream>;

  TunnelSession(asio::io_context& io, Ws& ws, PtyFactory factory, std::function<bool()> stopping)
      : io_(io),
        ws_(ws),
        sessions_(std::move(factory)),
        stopping_(std::move(stopping)),
        liveness_(kPingInterval, kDeadAfter, clock::now()),
        timer_(io) {}

  TunnelResult run() {
    const auto started = clock::now();
    send(false, hello_message(kVersion, kCommit));
    start_read();
    start_timer();
    io_.run();
    result_.connected_for = clock::now() - started;
    return result_;
  }

 private:
  struct Reader {
    Reader(asio::io_context& io, int fd) : descriptor(io, ::dup(fd)) {}
    asio::posix::stream_descriptor descriptor;
    std::array<char, 16 * 1024> buffer{};
    bool paused = false;
  };

  // ── WebSocket ───────────────────────────────────────────────────────────

  void start_read() {
    ws_.async_read(read_buffer_, [this](beast::error_code ec, std::size_t) { on_read(ec); });
  }

  void on_read(beast::error_code ec) {
    if (finished_) return;
    if (ec) {
      // A received close frame is authoritative even when the read then fails: the backend
      // (Kestrel) drops TCP after the close handshake without a TLS close_notify, which Beast
      // reports as stream_truncated rather than websocket::error::closed.
      if (ec == websocket::error::closed || ws_.reason().code != websocket::close_code::none) {
        const auto reason = std::string(ws_.reason().reason.c_str());
        if (reason == "token_expired")
          finish(TunnelEnd::TokenExpired, reason);
        else if (reason == "revoked")
          finish(TunnelEnd::Revoked, reason);
        else
          finish(TunnelEnd::Closed, "closed by backend: " + std::to_string(ws_.reason().code) + " " + reason);
      } else {
        finish(TunnelEnd::Closed, error_text(ec));
      }
      return;
    }

    liveness_.on_received(clock::now());
    const bool text = ws_.got_text();
    const auto data = beast::buffers_to_string(read_buffer_.data());
    read_buffer_.consume(read_buffer_.size());
    if (text)
      on_control(data);
    else
      on_data(data);
    start_read();
  }

  void send(bool binary, std::string message) {
    queued_bytes_ += message.size();
    outbox_.emplace_back(binary, std::move(message));
    if (!writing_) write_next();
  }

  void write_next() {
    if (finished_ || outbox_.empty()) {
      writing_ = false;
      return;
    }
    writing_ = true;
    ws_.text(!outbox_.front().first);
    ws_.async_write(asio::buffer(outbox_.front().second), [this](beast::error_code ec, std::size_t) {
      if (finished_) return;
      if (ec) {
        finish(TunnelEnd::Closed, "write failed: " + error_text(ec));
        return;
      }
      queued_bytes_ -= outbox_.front().second.size();
      outbox_.pop_front();
      if (queued_bytes_ < kResumeBytes) resume_readers();
      write_next();
    });
  }

  // ── Control messages from the backend ───────────────────────────────────

  void on_control(const std::string& data) {
    nlohmann::json msg;
    try {
      msg = nlohmann::json::parse(data);
    } catch (const nlohmann::json::exception&) {
      log::warn("ignoring malformed control message");
      return;
    }
    const auto type = msg.value("type", "");
    if (type == "pong") return;

    const auto id = parse_uuid(msg.value("sessionId", ""));
    if (!id) return;

    if (type == "open") {
      open_session(*id, msg.value("cols", 80), msg.value("rows", 24));
    } else if (type == "resize") {
      sessions_.resize(*id, msg.value("cols", 80), msg.value("rows", 24));
    } else if (type == "close") {
      log::info("session " + format_uuid(*id) + " closed by backend");
      readers_.erase(*id);
      sessions_.close(*id);
    }
  }

  void open_session(const Uuid& id, int cols, int rows) {
    auto* pty = sessions_.open(id, cols, rows);
    if (!pty) {
      send(false, exited_message(id, std::nullopt));
      return;
    }
    log::info("session " + format_uuid(id) + " opened (" + std::to_string(cols) + "x" + std::to_string(rows) + ")");
    auto reader = std::make_shared<Reader>(io_, pty->fd());
    readers_[id] = reader;
    read_pty(id, reader);
  }

  void on_data(const std::string& frame) {
    Uuid id;
    std::string_view payload;
    if (!decode_data_frame(frame, id, payload)) return;
    if (auto* pty = sessions_.find(id)) write_all(pty->fd(), payload);
  }

  // ── Shell output ────────────────────────────────────────────────────────

  void read_pty(const Uuid& id, const std::shared_ptr<Reader>& reader) {
    // The shared_ptr keeps the buffer alive until the handler runs, even if the session is closed
    // (and the reader erased) while the read is pending.
    reader->descriptor.async_read_some(asio::buffer(reader->buffer), [this, id, reader](beast::error_code ec, std::size_t n) {
      if (finished_ || ec == asio::error::operation_aborted) return;
      auto it = readers_.find(id);
      if (it == readers_.end() || it->second != reader) return;

      if (ec) {
        // EIO/EOF: every process holding the terminal has exited.
        shell_exited(id);
        return;
      }
      send(true, encode_data_frame(id, std::string_view(reader->buffer.data(), n)));
      if (queued_bytes_ >= kPauseBytes)
        reader->paused = true;
      else
        read_pty(id, reader);
    });
  }

  void resume_readers() {
    for (auto& [id, reader] : readers_) {
      if (reader->paused) {
        reader->paused = false;
        read_pty(id, reader);
      }
    }
  }

  void shell_exited(const Uuid& id) {
    readers_.erase(id);
    std::optional<int> code;
    if (auto pty = sessions_.release(id)) {
      code = pty->wait_exit(std::chrono::milliseconds(500));
      pty->terminate();
    }
    log::info("session " + format_uuid(id) + " shell exited" + (code ? " (" + std::to_string(*code) + ")" : ""));
    send(false, exited_message(id, code));
  }

  // ── Keepalive / shutdown ────────────────────────────────────────────────

  void start_timer() {
    timer_.expires_after(std::chrono::seconds(1));
    timer_.async_wait([this](beast::error_code ec) {
      if (finished_ || ec) return;
      const auto now = clock::now();
      if (stopping_()) {
        finish(TunnelEnd::Stopped, "");
        return;
      }
      if (liveness_.is_dead(now)) {
        finish(TunnelEnd::Dead, "nothing received for " + std::to_string(kDeadAfter.count()) + "s");
        return;
      }
      if (liveness_.ping_due(now)) {
        send(false, ping_message());
        liveness_.on_ping_sent(now);
      }
      start_timer();
    });
  }

  void finish(TunnelEnd end, std::string detail) {
    if (finished_) return;
    finished_ = true;
    result_.end = end;
    result_.detail = std::move(detail);

    // Nobody can see these shells any more; don't leave them running.
    readers_.clear();
    sessions_.close_all();

    timer_.cancel();
    beast::error_code ignored;
    beast::get_lowest_layer(ws_).socket().shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
    beast::get_lowest_layer(ws_).close();
    io_.stop();
  }

  asio::io_context& io_;
  Ws& ws_;
  SessionManager sessions_;
  std::function<bool()> stopping_;
  Liveness liveness_;
  asio::steady_timer timer_;

  beast::flat_buffer read_buffer_;
  std::deque<std::pair<bool, std::string>> outbox_;  // (binary, message)
  std::size_t queued_bytes_ = 0;
  bool writing_ = false;
  std::map<Uuid, std::shared_ptr<Reader>> readers_;

  bool finished_ = false;
  TunnelResult result_{TunnelEnd::Closed, {}, {}};
};

}  // namespace

HttpBackend::HttpBackend(const AgentConfig& cfg, PtyFactory pty_factory, std::function<bool()> stopping)
    : cfg_(cfg), tls_(make_tls_context(cfg)), pty_factory_(std::move(pty_factory)), stopping_(std::move(stopping)) {}

LoginResult HttpBackend::login() {
  try {
    auto secret = read_secret();
    if (secret.empty()) secret = cfg_.secret;
    const auto body = nlohmann::json{{"robotId", cfg_.robot_id}, {"secret", secret}}.dump();
    const auto res = https_request(tls_, cfg_, "POST", "/api/robot/login", body);
    if (res.status == 401 || res.status == 403) return {LoginResult::Status::Rejected, "", "HTTP " + std::to_string(res.status)};
    if (res.status != 200) return {LoginResult::Status::Failed, "", "HTTP " + std::to_string(res.status)};

    const auto json = nlohmann::json::parse(res.body, nullptr, /*allow_exceptions=*/false);
    if (!json.is_object() || !json.contains("token") || !json["token"].is_string())
      return {LoginResult::Status::Failed, "", "login response has no token"};
    return {LoginResult::Status::Ok, json["token"].get<std::string>(), ""};
  } catch (const std::exception& e) {
    return {LoginResult::Status::Failed, "", error_text(e)};
  }
}

TunnelResult HttpBackend::run_tunnel(const std::string& token) {
  asio::io_context io;
  TunnelSession::Ws ws(io, tls_);

  try {
    connect_tls(ws.next_layer(), cfg_.backend, proxy_for(cfg_.backend.host), std::chrono::seconds(15));
  } catch (const std::exception& e) {
    return {TunnelEnd::ConnectFailed, {}, error_text(e)};
  }

  // Tunnel I/O deadlines come from Liveness, not the TCP layer.
  beast::get_lowest_layer(ws).expires_never();
  websocket::stream_base::timeout timeouts{};
  timeouts.handshake_timeout = std::chrono::seconds(15);
  timeouts.idle_timeout = websocket::stream_base::none();
  timeouts.keep_alive_pings = false;
  ws.set_option(timeouts);
  ws.set_option(websocket::stream_base::decorator([&token](websocket::request_type& req) {
    req.set(http::field::authorization, "Bearer " + token);
    req.set(http::field::user_agent, std::string("bloom-robot-agent/") + kVersion);
  }));
  ws.read_message_max(1 << 20);

  websocket::response_type res;
  beast::error_code ec;
  ws.handshake(res, cfg_.backend.authority(), cfg_.backend.base_path + "/ws/robot-agent", ec);
  if (ec) {
    const auto status = res.result_int();
    if (status == 401 || status == 403) return {TunnelEnd::AuthRejected, {}, "HTTP " + std::to_string(status)};
    if (status != 0) return {TunnelEnd::ConnectFailed, {}, "WebSocket upgrade refused: HTTP " + std::to_string(status)};
    return {TunnelEnd::ConnectFailed, {}, error_text(ec)};
  }

  log::info("tunnel up to " + cfg_.backend.authority() + " (agent " + kVersion + ")");
  TunnelSession session(io, ws, pty_factory_, stopping_);
  return session.run();
}

}  // namespace bloom_agent
