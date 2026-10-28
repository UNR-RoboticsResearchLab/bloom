// Real forkpty: proves the shell gets a controlling terminal of the requested size, that resizes
// reach it, and that terminate() reaps the process group.
#include "pty_process.h"

#include <gtest/gtest.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <string>

using namespace bloom_agent;

namespace {
std::string read_until(int fd, const std::string& needle, std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
  std::string out;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (out.find(needle) == std::string::npos && std::chrono::steady_clock::now() < deadline) {
    pollfd p{fd, POLLIN, 0};
    if (poll(&p, 1, 50) <= 0) continue;
    char buf[4096];
    const ssize_t n = read(fd, buf, sizeof(buf));
    if (n <= 0) break;
    out.append(buf, static_cast<size_t>(n));
  }
  return out;
}

void type(IPty& pty, const std::string& s) { ASSERT_EQ(write(pty.fd(), s.data(), s.size()), static_cast<ssize_t>(s.size())); }
}  // namespace

TEST(PosixPty, ShellSeesTerminalSizeAndResizes) {
  auto pty = PosixPty::spawn("/bin/sh", 100, 30);
  type(*pty, "stty size; echo MARK1\n");
  EXPECT_NE(read_until(pty->fd(), "30 100").find("30 100"), std::string::npos);

  pty->resize(132, 43);
  type(*pty, "stty size; echo MARK2\n");
  EXPECT_NE(read_until(pty->fd(), "43 132").find("43 132"), std::string::npos);
}

TEST(PosixPty, ReportsExitCode) {
  auto pty = PosixPty::spawn("/bin/sh", 80, 24);
  type(*pty, "exit 7\n");
  read_until(pty->fd(), "\x04", std::chrono::milliseconds(500));
  EXPECT_EQ(pty->wait_exit(std::chrono::seconds(5)), 7);
}

// Same semantics as an ssh session dropping: everything on the terminal gets SIGHUP, including
// background jobs in their own process groups. (nohup'd jobs survive, as over ssh; stopping the
// agent's systemd unit kills everything in its cgroup regardless.)
TEST(PosixPty, TerminateHangsUpBackgroundJobs) {
  auto pty = PosixPty::spawn("/bin/bash", 80, 24);
  const pid_t shell = pty->pid();
  // The terminal echoes the typed line back; quoting the marker means only the output has "BG=".
  type(*pty, "sleep 300 & echo B''G=$!; echo E''ND\n");
  const auto all = read_until(pty->fd(), "END");
  const auto pos = all.find("BG=");
  ASSERT_NE(pos, std::string::npos) << all;
  const pid_t bg = std::stoi(all.substr(pos + 3));

  pty->terminate();
  EXPECT_NE(kill(shell, 0), 0);
  // The background job got the hangup too (it may linger a moment as a zombie of init).
  for (int i = 0; i < 50 && kill(bg, 0) == 0; ++i) usleep(20000);
  EXPECT_NE(kill(bg, 0), 0);
}

TEST(PosixPty, DoesNotLeakSecretsIntoShell) {
  setenv("BLOOM_ROBOT_SECRET", "hunter2", 1);
  auto pty = PosixPty::spawn("/bin/sh", 80, 24);
  unsetenv("BLOOM_ROBOT_SECRET");
  type(*pty, "echo \"S=[${BLOOM_ROBOT_SECRET}] T=[$TERM]\"\n");
  const auto out = read_until(pty->fd(), "T=[xterm");
  EXPECT_NE(out.find("S=[]"), std::string::npos) << out;
  EXPECT_NE(out.find("T=[xterm-256color]"), std::string::npos) << out;
}
