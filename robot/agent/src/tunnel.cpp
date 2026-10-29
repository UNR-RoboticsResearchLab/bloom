#include "tunnel.h"

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
#include "pty_writer.h"
#include "sessions.h"
#include "signalr.h"
#include "version.h"

namespace bloom_agent {

namespace http = beast::http;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using clock = std::chrono::steady_clock;

namespace {

// SignalR's server pings us every 15 s and drops a client that has been silent for the backend's
// ClientTimeoutInterval (75 s by default), so we send something at least that often too.
constexpr auto kPingInterval = std::chrono::seconds(15);
constexpr auto kDeadAfter = std::chrono::seconds(60);
// Stop reading shell output while this much is waiting to go out (e.g. `cat` of a huge file over a
// slow link), so memory stays bounded; resume below kResumeBytes.
constexpr std::size_t kPauseBytes = 1 << 20;
constexpr std::size_t kResumeBytes = 256 << 10;

// One established tunnel. Everything runs on one thread inside io.run(): WebSocket reads and
// writes, every PTY's output, and the keepalive timer.
class TunnelSession {
 public:
  using Ws = websocket::stream<TlsStream>;

  TunnelSession(asio::io_context& io, Ws& ws, PtyFactory factory, SecretApplier applier, std::function<bool()> stopping)
      : io_(io),
        ws_(ws),
        sessions_(std::move(factory)),
        secret_applier_(std::move(applier)),
        stopping_(std::move(stopping)),
        liveness_(kPingInterval, kDeadAfter, clock::now()),
        timer_(io) {
    ws_.text(true);  // SignalR's JSON protocol is text frames throughout
  }

  TunnelResult run() {
    const auto started = clock::now();
    // The first thing on a SignalR connection; Hello follows once the server accepts it.
    send(signalr::handshake_request());
    start_read();
    start_timer();
    io_.run();
    result_.connected_for = clock::now() - started;
    return result_;
  }

 private:
  struct Reader {
    Reader(asio::io_context& io, int fd) : descriptor(io, ::dup(fd)), input(PtyWriter::create(io, fd)) {}
    ~Reader() { input->cancel(); }
    asio::posix::stream_descriptor descriptor;
    std::shared_ptr<PtyWriter> input;  // keystrokes in: asynchronous, see pty_writer.h
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
      if (ec == websocket::error::closed || ws_.reason().code != websocket::close_code::none)
        finish(TunnelEnd::Closed, "closed by backend: " + std::to_string(ws_.reason().code) + " " + std::string(ws_.reason().reason.c_str()));
      else
        finish(TunnelEnd::Closed, error_text(ec));
      return;
    }

    liveness_.on_received(clock::now());
    records_.feed(beast::buffers_to_string(read_buffer_.data()));
    read_buffer_.consume(read_buffer_.size());
    // A message may hold several records, and handling one can end the tunnel.
    while (!finished_) {
      const auto record = records_.next();
      if (!record) break;
      on_record(*record);
    }
    if (!finished_) start_read();
  }

  void send(std::string message) {
    queued_bytes_ += message.size();
    outbox_.push_back(std::move(message));
    if (!writing_) write_next();
  }

  void write_next() {
    if (finished_ || outbox_.empty()) {
      writing_ = false;
      return;
    }
    writing_ = true;
    ws_.async_write(asio::buffer(outbox_.front()), [this](beast::error_code ec, std::size_t) {
      if (finished_) return;
      if (ec) {
        finish(TunnelEnd::Closed, "write failed: " + error_text(ec));
        return;
      }
      queued_bytes_ -= outbox_.front().size();
      outbox_.pop_front();
      if (queued_bytes_ < kResumeBytes) resume_readers();
      write_next();
    });
  }

  // ── Messages from the backend ───────────────────────────────────────────

  void on_record(const std::string& record) {
    if (!handshaken_) {
      const auto reply = signalr::parse_handshake_response(record);
      if (!reply.ok) {
        finish(TunnelEnd::ConnectFailed, "SignalR handshake refused: " + reply.error);
        return;
      }
      handshaken_ = true;
      send(hello_message(kVersion, kCommit));
      return;
    }

    const auto message = signalr::parse_message(record);
    if (!message) {
      log::warn("ignoring a malformed SignalR message");
      return;
    }
    switch (message->type) {
      case signalr::MessageType::Invocation:
        on_invocation(message->target, message->arguments);
        break;
      case signalr::MessageType::Close:
        finish(TunnelEnd::Closed, "the backend closed the connection" + (message->error.empty() ? "" : ": " + message->error));
        break;
      default:
        break;  // Ping: it already counted as liveness. Completions: we never ask for one.
    }
  }

  void on_invocation(const std::string& target, const nlohmann::json& args) {
    if (target == "SetSecret") {
      on_set_secret(args);
    } else if (target == "Disconnecting") {
      on_disconnecting(arg_string(args, 0).value_or(""));
    } else if (target == "Open") {
      const auto id = arg_uuid(args, 0);
      if (id) open_session(*id, arg_int(args, 1).value_or(80), arg_int(args, 2).value_or(24));
    } else if (target == "Input") {
      const auto id = arg_uuid(args, 0);
      const auto bytes = arg_bytes(args, 1);
      if (id && bytes) {
        const auto it = readers_.find(*id);
        if (it != readers_.end()) it->second->input->write(*bytes);
      }
    } else if (target == "Resize") {
      const auto id = arg_uuid(args, 0);
      if (id) sessions_.resize(*id, arg_int(args, 1).value_or(80), arg_int(args, 2).value_or(24));
    } else if (target == "Close") {
      const auto id = arg_uuid(args, 0);
      if (id) {
        log::info("session " + format_uuid(*id) + " closed by backend");
        readers_.erase(*id);
        sessions_.close(*id);
      }
    }
  }

  // A hub can't attach a reason to dropping a connection, so the backend says why first. Act on it
  // now rather than waiting for the connection to die: the reason decides how we reconnect.
  void on_disconnecting(const std::string& reason) {
    if (reason == "token_expired")
      finish(TunnelEnd::TokenExpired, reason);
    else if (reason == "credentials_rotated")
      finish(TunnelEnd::CredentialsRotated, reason);
    else if (reason == "revoked")
      finish(TunnelEnd::Revoked, reason);
    else
      finish(TunnelEnd::Closed, "dropped by backend: " + reason);  // robot_deleted, replaced, ...
  }

  // The backend pushes a rotated secret. Apply it to disk, confirm, and the backend then drops the
  // connection so we log in again with it. Blocks the loop for the script's runtime (seconds).
  void on_set_secret(const nlohmann::json& args) {
    const auto request_id = request_id_of(args);
    if (!request_id) {
      log::warn("ignoring SetSecret without a valid requestId");
      return;
    }
    const auto request = parse_set_secret(args);
    if (!request) {
      send(secret_applied_message(*request_id, false, "the pushed secret was malformed"));
      return;
    }
    if (secret_is_externally_managed()) {
      send(secret_applied_message(*request_id, false,
                                  "this robot's secret comes from systemd credentials and can't be replaced remotely"));
      return;
    }
    log::info("applying a pushed secret");
    const auto result = secret_applier_(request->secret);
    log::info(result.ok ? "pushed secret applied" : "pushed secret NOT applied: " + result.message);
    send(secret_applied_message(*request_id, result.ok, result.message));
  }

  void open_session(const Uuid& id, int cols, int rows) {
    auto* pty = sessions_.open(id, cols, rows);
    if (!pty) {
      send(exited_message(id, std::nullopt));
      return;
    }
    log::info("session " + format_uuid(id) + " opened (" + std::to_string(cols) + "x" + std::to_string(rows) + ")");
    auto reader = std::make_shared<Reader>(io_, pty->fd());
    readers_[id] = reader;
    read_pty(id, reader);
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
      send(output_message(id, std::string_view(reader->buffer.data(), n)));
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
    send(exited_message(id, code));
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
        send(signalr::ping());
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
  SecretApplier secret_applier_;
  std::function<bool()> stopping_;
  Liveness liveness_;
  asio::steady_timer timer_;

  beast::flat_buffer read_buffer_;
  std::deque<std::string> outbox_;
  signalr::RecordParser records_;
  bool handshaken_ = false;
  std::size_t queued_bytes_ = 0;
  bool writing_ = false;
  std::map<Uuid, std::shared_ptr<Reader>> readers_;

  bool finished_ = false;
  TunnelResult result_{TunnelEnd::Closed, {}, {}};
};

}  // namespace

HttpBackend::HttpBackend(const AgentConfig& cfg, PtyFactory pty_factory, SecretApplier secret_applier,
                         std::function<bool()> stopping)
    : cfg_(cfg),
      tls_(make_tls_context(cfg)),
      pty_factory_(std::move(pty_factory)),
      secret_applier_(std::move(secret_applier)),
      stopping_(std::move(stopping)) {}

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
  ws.handshake(res, cfg_.backend.authority(), cfg_.backend.base_path + "/hubs/robot-agent", ec);
  if (ec) {
    const auto status = res.result_int();
    if (status == 401 || status == 403) return {TunnelEnd::AuthRejected, {}, "HTTP " + std::to_string(status)};
    if (status != 0) return {TunnelEnd::ConnectFailed, {}, "WebSocket upgrade refused: HTTP " + std::to_string(status)};
    return {TunnelEnd::ConnectFailed, {}, error_text(ec)};
  }

  log::info("connected to " + cfg_.backend.authority() + " (agent " + kVersion + ")");
  TunnelSession session(io, ws, pty_factory_, secret_applier_, stopping_);
  return session.run();
}

}  // namespace bloom_agent
