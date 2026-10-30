#include "supervisor.h"

#include "log.h"

namespace bloom_agent {

const char* to_string(TunnelEnd end) {
  switch (end) {
    case TunnelEnd::ConnectFailed: return "connect_failed";
    case TunnelEnd::AuthRejected: return "auth_rejected";
    case TunnelEnd::Closed: return "closed";
    case TunnelEnd::TokenExpired: return "token_expired";
    case TunnelEnd::CredentialsRotated: return "credentials_rotated";
    case TunnelEnd::Revoked: return "revoked";
    case TunnelEnd::Dead: return "dead";
    case TunnelEnd::Stopped: return "stopped";
  }
  return "unknown";
}

std::chrono::milliseconds Supervisor::step() {
  // Always log in fresh: tokens are short-lived, and a 401 on the upgrade then unambiguously means
  // the robot was revoked rather than "token went stale while we slept".
  const auto login = backend_.login();
  if (login.status != LoginResult::Status::Ok) {
    const auto delay = backoff_.next();
    const char* what = login.status == LoginResult::Status::Rejected ? "rejected" : "failed";
    log::warn(std::string("login ") + what + ": " + login.detail + "; retrying in " +
              std::to_string(delay.count()) + "ms");
    return delay;
  }

  const auto result = backend_.run_tunnel(login.token);
  const auto up_for = std::chrono::duration_cast<std::chrono::seconds>(result.connected_for);
  if (result.end == TunnelEnd::Stopped) return std::chrono::milliseconds(0);

  if (result.end == TunnelEnd::CredentialsRotated) {
    backoff_.reset();
    log::info("credentials rotated; reconnecting with the new secret");
    return std::chrono::milliseconds(0);
  }

  if (result.end == TunnelEnd::TokenExpired) {
    // Routine: the backend ends every tunnel at token expiry. Reconnect straight away.
    backoff_.reset();
    log::info("tunnel reached token expiry after " + std::to_string(up_for.count()) + "s; reconnecting");
    return std::chrono::milliseconds(0);
  }

  if (result.connected_for >= kStableAfter) backoff_.reset();
  const auto delay = backoff_.next();
  log::warn(std::string("tunnel ended (") + to_string(result.end) + (result.detail.empty() ? "" : ": " + result.detail) +
            ") after " + std::to_string(up_for.count()) + "s; retrying in " + std::to_string(delay.count()) + "ms");
  return delay;
}

void Supervisor::run(const std::function<bool()>& stopping,
                     const std::function<void(std::chrono::milliseconds)>& sleep) {
  while (!stopping()) {
    const auto delay = step();
    if (!stopping() && delay.count() > 0) sleep(delay);
  }
}

}  // namespace bloom_agent
