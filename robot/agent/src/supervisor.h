// The agent's outer loop: log in, hold the tunnel, and when it drops decide how long to wait
// before trying again. The network side sits behind IBackend so this logic is tested with fakes.
#pragma once

#include <chrono>
#include <functional>
#include <string>

#include "backoff.h"

namespace bloom_agent {

struct LoginResult {
  enum class Status { Ok, Rejected, Failed };  // Rejected: backend said 401 (revoked / wrong secret)
  Status status;
  std::string token;
  std::string detail;
};

enum class TunnelEnd {
  ConnectFailed,  // never got a WebSocket up (DNS, TCP, proxy, TLS, HTTP error)
  AuthRejected,   // upgrade refused with 401/403
  Closed,         // backend or network closed an established tunnel
  TokenExpired,   // backend closed it at JWT expiry -- expected every Jwt:ExpireMinutes
  Revoked,        // backend closed it because the robot was revoked
  Dead,           // nothing received for too long
  Stopped,        // agent is shutting down
};
const char* to_string(TunnelEnd end);

struct TunnelResult {
  TunnelEnd end;
  std::chrono::steady_clock::duration connected_for{};
  std::string detail;
};

class IBackend {
 public:
  virtual ~IBackend() = default;
  virtual LoginResult login() = 0;
  // Blocks for the life of the tunnel.
  virtual TunnelResult run_tunnel(const std::string& token) = 0;
};

class Supervisor {
 public:
  // A tunnel that stayed up this long counts as healthy and resets the backoff.
  static constexpr std::chrono::seconds kStableAfter{60};

  Supervisor(IBackend& backend, Backoff& backoff) : backend_(backend), backoff_(backoff) {}

  // One login + tunnel attempt. Returns how long to wait before the next.
  std::chrono::milliseconds step();

  // step() until `stopping`, sleeping between attempts with `sleep` (which may return early on stop).
  void run(const std::function<bool()>& stopping, const std::function<void(std::chrono::milliseconds)>& sleep);

 private:
  IBackend& backend_;
  Backoff& backoff_;
};

}  // namespace bloom_agent
