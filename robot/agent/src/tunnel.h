// The real IBackend: logs in over HTTPS and holds the WebSocket tunnel, running shells on request.
#pragma once

#include <atomic>
#include <functional>

#include "config.h"
#include "net.h"
#include "pty_process.h"
#include "supervisor.h"

namespace bloom_agent {

class HttpBackend : public IBackend {
 public:
  HttpBackend(const AgentConfig& cfg, PtyFactory pty_factory, std::function<bool()> stopping);

  LoginResult login() override;
  TunnelResult run_tunnel(const std::string& token) override;

 private:
  const AgentConfig& cfg_;
  ssl::context tls_;
  PtyFactory pty_factory_;
  std::function<bool()> stopping_;
};

}  // namespace bloom_agent
