// Runs real scripts: proves the secret reaches the script on stdin only, results and failures are
// reported, and a hung script is killed.
#include "secret_apply.h"

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <sstream>

using namespace bloom_agent;

namespace {
class SecretApply : public ::testing::Test {
 protected:
  void SetUp() override {
    char tmpl[] = "/tmp/secret_apply_XXXXXX";
    dir_ = mkdtemp(tmpl);
  }
  void TearDown() override { (void)!system(("rm -rf " + dir_).c_str()); }

  std::string script(const std::string& body) {
    const std::string path = dir_ + "/script.sh";
    std::ofstream(path) << "#!/bin/sh\n" << body << "\n";
    chmod(path.c_str(), 0755);
    return path;
  }
  std::string read(const std::string& name) {
    std::ifstream in(dir_ + "/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
  }
  ApplyOptions opts(const std::string& command, int timeout_s = 10) {
    ApplyOptions o;
    o.command = command;
    o.use_sudo = false;
    o.timeout = std::chrono::seconds(timeout_s);
    return o;
  }
  std::string dir_;
};
}  // namespace

TEST_F(SecretApply, SecretArrivesOnStdinNotArgv) {
  const auto cmd = script("cat > " + dir_ + "/stdin; echo \"$@\" > " + dir_ + "/args; echo done");
  const auto r = apply_secret(opts(cmd), "Xq3-secretValue_123");
  EXPECT_TRUE(r.ok) << r.message;
  EXPECT_EQ(r.message, "done");
  EXPECT_EQ(read("stdin"), "Xq3-secretValue_123\n");
  EXPECT_EQ(read("args"), "--secret-stdin --no-restart-agent\n");
  EXPECT_EQ(read("args").find("Xq3"), std::string::npos);
}

TEST_F(SecretApply, FailureReportsLastLine) {
  const auto r = apply_secret(opts(script("cat >/dev/null; echo noise; echo 'cannot write /etc/bloom-agent/secret' >&2; exit 3")), "abcdefghijklmnop");
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.message, "cannot write /etc/bloom-agent/secret");
}

TEST_F(SecretApply, SilentFailureReportsExitCode) {
  const auto r = apply_secret(opts(script("exit 9")), "abcdefghijklmnop");
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.message, "exit code 9");
}

TEST_F(SecretApply, ScriptThatNeverReadsStdinDoesNotKillTheAgent) {
  signal(SIGPIPE, SIG_DFL);  // the agent ignores it; the helper must cope either way
  const auto r = apply_secret(opts(script("exit 0")), "abcdefghijklmnop");
  EXPECT_TRUE(r.ok);
}

TEST_F(SecretApply, SecretIsRedactedIfTheScriptEchoesIt) {
  const auto r = apply_secret(opts(script("read s; echo \"bad: $s\"; exit 1")), "abcdefghijklmnop");
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.message.find("abcdefghijklmnop"), std::string::npos);
  EXPECT_EQ(r.message, "bad: ***");
}

TEST_F(SecretApply, MissingScriptIsReported) {
  const auto r = apply_secret(opts(dir_ + "/nope.sh"), "abcdefghijklmnop");
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.message.empty());
}

TEST_F(SecretApply, HungScriptIsKilledAtTheTimeout) {
  const auto start = std::chrono::steady_clock::now();
  const auto r = apply_secret(opts(script("cat >/dev/null; sleep 61 & echo $! > " + dir_ + "/pid; wait"), /*timeout_s=*/1), "abcdefghijklmnop");
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.message, "timed out applying the secret");
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(5));

  // What the script started was killed too, not just the script.
  const pid_t child = std::stoi(read("pid"));
  for (int i = 0; i < 50 && kill(child, 0) == 0; ++i) usleep(20000);
  EXPECT_NE(kill(child, 0), 0);
}
