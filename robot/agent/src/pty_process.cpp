#include "pty_process.h"

#include <dirent.h>
#include <fcntl.h>
#include <pty.h>
#include <pwd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <system_error>
#include <thread>
#include <vector>

extern char** environ;

namespace bloom_agent {
namespace {
// Signals every process whose session id is `sid` -- everything that has this terminal as its
// controlling tty, including background jobs in other process groups, which kill(-pgid) misses.
void signal_session(pid_t sid, int sig) {
  DIR* proc = opendir("/proc");
  if (!proc) return;
  while (const dirent* entry = readdir(proc)) {
    const pid_t pid = static_cast<pid_t>(std::strtol(entry->d_name, nullptr, 10));
    if (pid <= 0) continue;
    std::ifstream stat(std::string("/proc/") + entry->d_name + "/stat");
    std::string line;
    if (!std::getline(stat, line)) continue;
    // Fields after the parenthesised comm (which may itself contain spaces/parens):
    // state ppid pgrp session ...
    const auto close_paren = line.rfind(')');
    if (close_paren == std::string::npos) continue;
    std::istringstream rest(line.substr(close_paren + 1));
    char state;
    pid_t ppid, pgrp, session;
    if (rest >> state >> ppid >> pgrp >> session && session == sid) kill(pid, sig);
  }
  closedir(proc);
}

// Never hand these to the shell: the tunnel's credentials have no business in an admin's session.
bool hidden_from_shell(const char* entry) {
  for (const char* prefix : {"BLOOM_ROBOT_SECRET=", "CREDENTIALS_DIRECTORY=", "INVOCATION_ID=", "NOTIFY_SOCKET="})
    if (std::strncmp(entry, prefix, std::strlen(prefix)) == 0) return true;
  return false;
}
}  // namespace

std::string default_shell() {
  if (const passwd* pw = getpwuid(getuid()); pw && pw->pw_shell && *pw->pw_shell) return pw->pw_shell;
  return "/bin/bash";
}

std::unique_ptr<PosixPty> PosixPty::spawn(const std::string& shell, int cols, int rows) {
  // Everything the child needs is built before fork(): only async-signal-safe calls happen between
  // fork and exec.
  const auto slash = shell.rfind('/');
  const std::string argv0 = "-" + (slash == std::string::npos ? shell : shell.substr(slash + 1));
  std::vector<char*> argv{const_cast<char*>(argv0.c_str()), nullptr};

  std::vector<std::string> env_storage{"TERM=xterm-256color"};
  for (char** e = environ; *e; ++e)
    if (!hidden_from_shell(*e) && std::strncmp(*e, "TERM=", 5) != 0) env_storage.emplace_back(*e);
  std::vector<char*> envp;
  for (auto& s : env_storage) envp.push_back(s.data());
  envp.push_back(nullptr);

  const passwd* pw = getpwuid(getuid());
  const std::string home = pw && pw->pw_dir ? pw->pw_dir : "/";

  winsize size{};
  size.ws_col = static_cast<unsigned short>(cols);
  size.ws_row = static_cast<unsigned short>(rows);

  int master = -1;
  const pid_t pid = forkpty(&master, nullptr, nullptr, &size);
  if (pid < 0) throw std::system_error(errno, std::generic_category(), "forkpty");

  if (pid == 0) {
    // Don't leak the TLS socket or anything else into the shell.
    close_range(3, ~0U, 0);
    if (chdir(home.c_str()) != 0 && chdir("/") != 0) _exit(126);
    execve(shell.c_str(), argv.data(), envp.data());
    _exit(127);
  }

  fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
  fcntl(master, F_SETFD, FD_CLOEXEC);
  return std::unique_ptr<PosixPty>(new PosixPty(master, pid));
}

PosixPty::~PosixPty() { terminate(); }

void PosixPty::resize(int cols, int rows) {
  if (master_ < 0) return;
  winsize size{};
  size.ws_col = static_cast<unsigned short>(cols);
  size.ws_row = static_cast<unsigned short>(rows);
  ioctl(master_, TIOCSWINSZ, &size);
}

bool PosixPty::reap(bool block) {
  if (reaped_) return true;
  int status = 0;
  const pid_t r = waitpid(pid_, &status, block ? 0 : WNOHANG);
  if (r == pid_ || (r < 0 && errno == ECHILD)) {
    reaped_ = true;
    if (r == pid_ && WIFEXITED(status)) exit_code_ = WEXITSTATUS(status);
  }
  return reaped_;
}

std::optional<int> PosixPty::wait_exit(std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!reap(false) && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  return exit_code_;
}

void PosixPty::terminate() {
  if (!reaped_) {
    // Like an ssh hangup: everything on this terminal gets SIGHUP. Jobs started with nohup ignore
    // it and survive, as they would over ssh. The shell is the session leader (forkpty), so its pid
    // is the session id. Escalation to SIGKILL only targets the shell's own process group.
    signal_session(pid_, SIGHUP);
    kill(-pid_, SIGHUP);
    wait_exit(std::chrono::milliseconds(300));
    if (!reaped_) {
      kill(-pid_, SIGKILL);
      kill(pid_, SIGKILL);
      reap(true);
    }
  }
  if (master_ >= 0) {
    close(master_);
    master_ = -1;
  }
}

}  // namespace bloom_agent
