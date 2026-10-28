// robot-agent: holds an outbound TLS WebSocket tunnel to the Bloom backend so admins can open a
// shell on this robot from the fleet dashboard, without any inbound port or sshd exposure.
//
//   robot-agent [run] [--config PATH]   run the agent (what the systemd unit does)
//   robot-agent doctor [--config PATH]  diagnose connectivity from this network
//   robot-agent version

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <unistd.h>

#include "config.h"
#include "doctor.h"
#include "log.h"
#include "pty_process.h"
#include "secret_apply.h"
#include "supervisor.h"
#include "tunnel.h"
#include "version.h"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

int usage() {
  std::fprintf(stderr, "usage: robot-agent [run|doctor|version] [--config PATH]\n");
  return 64;  // EX_USAGE
}
}  // namespace

int main(int argc, char** argv) {
  using namespace bloom_agent;

  std::string command = "run";
  std::string config_path = std::getenv("BLOOM_AGENT_CONFIG") ? std::getenv("BLOOM_AGENT_CONFIG") : kDefaultConfigPath;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc)
      config_path = argv[++i];
    else if (argv[i][0] != '-')
      command = argv[i];
    else
      return usage();
  }

  if (command == "version") {
    std::printf("%s %s\n", kVersion, kCommit);
    return 0;
  }
  if (command == "doctor") return run_doctor(config_path);
  if (command != "run") return usage();

  AgentConfig cfg;
  try {
    cfg = load_config(config_path);
  } catch (const std::exception& e) {
    log::error(std::string("config: ") + e.what());
    return 78;  // EX_CONFIG
  }

  std::signal(SIGTERM, on_signal);
  std::signal(SIGINT, on_signal);
  std::signal(SIGPIPE, SIG_IGN);
  auto stopping = [] { return g_stop != 0; };

  const std::string shell = cfg.shell.empty() ? default_shell() : cfg.shell;
  PtyFactory pty_factory = [shell](int cols, int rows) -> std::unique_ptr<IPty> { return PosixPty::spawn(shell, cols, rows); };

  std::mt19937_64 rng{std::random_device{}()};
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  Backoff backoff(std::chrono::seconds(1), std::chrono::seconds(60), [&] { return uniform(rng); });

  log::info(std::string("robot-agent ") + kVersion + " (" + kCommit + ") starting for robot " + cfg.robot_id + " -> " +
            cfg.backend.authority());

  ApplyOptions apply_options;
  apply_options.command = cfg.apply_secret_command;
  apply_options.use_sudo = geteuid() != 0;
  SecretApplier secret_applier = [apply_options](const std::string& secret) { return apply_secret(apply_options, secret); };

  HttpBackend backend(cfg, pty_factory, secret_applier, stopping);
  Supervisor supervisor(backend, backoff);
  supervisor.run(stopping, [&](std::chrono::milliseconds delay) {
    // Sleep in slices so SIGTERM is honoured promptly.
    const auto until = std::chrono::steady_clock::now() + delay;
    while (!stopping() && std::chrono::steady_clock::now() < until)
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  });

  log::info("robot-agent stopped");
  return 0;
}
