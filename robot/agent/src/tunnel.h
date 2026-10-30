// The real IBackend: logs in over HTTPS and holds the WebSocket tunnel, running shells on request.
#pragma once

#include <atomic>
#include <functional>

#include "config.h"
#include "net.h"
#include "ota_manager.h"
#include "pty_process.h"
#include "secret_apply.h"
#include "supervisor.h"

namespace bloom_agent {

class HttpBackend : public IBackend {
 public:
  HttpBackend(const AgentConfig& cfg, PtyFactory pty_factory, SecretApplier secret_applier, std::function<bool()> stopping,
              ota::OtaManager* ota = nullptr);

  LoginResult login() override;
  TunnelResult run_tunnel(const std::string& token) override;

 private:
  const AgentConfig& cfg_;
  ssl::context tls_;
  PtyFactory pty_factory_;
  SecretApplier secret_applier_;
  std::function<bool()> stopping_;
  ota::OtaManager* ota_;  // null when OTA is disabled
};

}  // namespace bloom_agent
