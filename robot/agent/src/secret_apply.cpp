#include "secret_apply.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

namespace bloom_agent {
namespace {

std::string last_line(const std::string& output, const std::string& secret) {
  std::string text = output;
  // Belt and braces: the script never prints the secret, but if it ever did, don't relay it.
  for (size_t pos; !secret.empty() && (pos = text.find(secret)) != std::string::npos;) text.replace(pos, secret.size(), "***");
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) text.pop_back();
  const auto nl = text.rfind('\n');
  std::string line = nl == std::string::npos ? text : text.substr(nl + 1);
  for (char& c : line)
    if (static_cast<unsigned char>(c) < 0x20) c = ' ';
  if (line.size() > 200) line.resize(200);
  return line;
}

void close_fd(int& fd) {
  if (fd >= 0) ::close(fd);
  fd = -1;
}

}  // namespace

ApplyResult apply_secret(const ApplyOptions& options, const std::string& secret) {
  std::vector<std::string> args;
  if (options.use_sudo) args = {"sudo", "-n"};
  args.insert(args.end(), {options.command, "--secret-stdin", "--no-restart-agent"});
  std::vector<char*> argv;
  for (auto& a : args) argv.push_back(a.data());
  argv.push_back(nullptr);

  int in_pipe[2], out_pipe[2];
  if (pipe2(in_pipe, O_CLOEXEC) != 0) return {false, std::string("pipe: ") + std::strerror(errno)};
  if (pipe2(out_pipe, O_CLOEXEC) != 0) {
    ::close(in_pipe[0]);
    ::close(in_pipe[1]);
    return {false, std::string("pipe: ") + std::strerror(errno)};
  }

  const pid_t pid = fork();
  if (pid < 0) {
    for (int fd : {in_pipe[0], in_pipe[1], out_pipe[0], out_pipe[1]}) ::close(fd);
    return {false, std::string("fork: ") + std::strerror(errno)};
  }
  if (pid == 0) {
    setpgid(0, 0);  // own process group, so a timeout can kill the script and everything it started
    dup2(in_pipe[0], STDIN_FILENO);
    dup2(out_pipe[1], STDOUT_FILENO);
    dup2(out_pipe[1], STDERR_FILENO);
    // Not the TLS socket, not the PTYs: just stdio.
    close_range(3, ~0U, 0);
    execvp(argv[0], argv.data());
    const char* msg = "cannot execute the provisioning script\n";
    (void)!write(STDERR_FILENO, msg, std::strlen(msg));
    _exit(127);
  }

  ::close(in_pipe[0]);
  ::close(out_pipe[1]);
  int stdin_fd = in_pipe[1];
  int stdout_fd = out_pipe[0];

  // The secret is far below the pipe buffer, so this can't block. A script that exits early gives
  // EPIPE, which must not kill the agent.
  struct sigaction ignore {}, previous {};
  ignore.sa_handler = SIG_IGN;
  sigaction(SIGPIPE, &ignore, &previous);
  const std::string input = secret + "\n";
  (void)!::write(stdin_fd, input.data(), input.size());
  sigaction(SIGPIPE, &previous, nullptr);
  close_fd(stdin_fd);

  std::string output;
  bool timed_out = false;
  const auto deadline = std::chrono::steady_clock::now() + options.timeout;
  while (true) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      timed_out = true;
      break;
    }
    pollfd p{stdout_fd, POLLIN, 0};
    const int r = poll(&p, 1, static_cast<int>(std::min<long long>(remaining.count(), 1000)));
    if (r < 0 && errno != EINTR) break;
    if (r > 0) {
      char buf[1024];
      const ssize_t n = ::read(stdout_fd, buf, sizeof(buf));
      if (n <= 0) break;  // EOF: the script (and anything it started) closed its output
      if (output.size() < 8192) output.append(buf, static_cast<size_t>(n));
    }
  }
  close_fd(stdout_fd);

  int status = 0;
  if (timed_out) {
    kill(-pid, SIGKILL);
    kill(pid, SIGKILL);
    waitpid(pid, &status, 0);
    return {false, "timed out applying the secret"};
  }
  while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }

  const auto message = last_line(output, secret);
  if (WIFEXITED(status) && WEXITSTATUS(status) == 0) return {true, message};
  if (!message.empty()) return {false, message};
  return {false, WIFEXITED(status) ? "exit code " + std::to_string(WEXITSTATUS(status)) : "killed by a signal"};
}

}  // namespace bloom_agent
