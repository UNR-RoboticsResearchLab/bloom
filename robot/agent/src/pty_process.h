// A login shell on a pseudo-terminal. IPty is the seam the tunnel and the tests use; PosixPty is
// the real thing (forkpty).
#pragma once

#include <sys/types.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace bloom_agent {

class IPty {
 public:
  virtual ~IPty() = default;
  // Master side: read terminal output from it, write keystrokes to it. Non-blocking.
  virtual int fd() const = 0;
  virtual void resize(int cols, int rows) = 0;
  // Hang up the shell and everything it started, and reap it. Idempotent.
  virtual void terminate() = 0;
  // Exit code once the shell has exited (waits up to `timeout`), nullopt if still running or
  // killed by a signal.
  virtual std::optional<int> wait_exit(std::chrono::milliseconds timeout) = 0;
};

using PtyFactory = std::function<std::unique_ptr<IPty>(int cols, int rows)>;

class PosixPty : public IPty {
 public:
  // Starts `shell` as a login shell (argv[0] = "-bash") in a new session with the PTY as its
  // controlling terminal. Throws std::system_error on failure.
  static std::unique_ptr<PosixPty> spawn(const std::string& shell, int cols, int rows);

  ~PosixPty() override;
  int fd() const override { return master_; }
  pid_t pid() const { return pid_; }
  void resize(int cols, int rows) override;
  void terminate() override;
  std::optional<int> wait_exit(std::chrono::milliseconds timeout) override;

 private:
  PosixPty(int master, pid_t pid) : master_(master), pid_(pid) {}
  bool reap(bool block);

  int master_;
  pid_t pid_;
  bool reaped_ = false;
  std::optional<int> exit_code_;
};

// The agent user's login shell from passwd, falling back to /bin/bash.
std::string default_shell();

}  // namespace bloom_agent
