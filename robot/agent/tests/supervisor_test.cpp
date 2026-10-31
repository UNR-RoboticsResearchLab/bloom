#include "supervisor.h"

#include <gtest/gtest.h>

#include <deque>

using namespace bloom_agent;
using namespace std::chrono_literals;
using ms = std::chrono::milliseconds;

namespace {
// Scripted backend: each login()/run_tunnel() pops the next canned result.
struct FakeBackend : IBackend {
  std::deque<LoginResult> logins;
  std::deque<TunnelResult> tunnels;
  int login_calls = 0;
  std::vector<std::string> tokens_used;

  LoginResult login() override {
    ++login_calls;
    auto r = logins.front();
    logins.pop_front();
    return r;
  }
  TunnelResult run_tunnel(const std::string& token) override {
    tokens_used.push_back(token);
    auto r = tunnels.front();
    tunnels.pop_front();
    return r;
  }
};

LoginResult ok(const std::string& token) { return {LoginResult::Status::Ok, token, ""}; }
LoginResult rejected() { return {LoginResult::Status::Rejected, "", "HTTP 401"}; }
LoginResult failed() { return {LoginResult::Status::Failed, "", "connection refused"}; }
TunnelResult ended(TunnelEnd e, std::chrono::steady_clock::duration up = 0s) { return {e, up, ""}; }

// Upper bound of jitter = ceiling, so delays are exactly the ceilings: 1s, 2s, 4s, ...
Backoff exact_backoff() { return Backoff(1000ms, 60000ms, [] { return 0.999999; }); }
}  // namespace

TEST(Supervisor, FailedLoginsBackOffExponentially) {
  FakeBackend backend;
  backend.logins = {failed(), failed(), rejected(), failed()};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  EXPECT_EQ(s.step(), 999ms);
  EXPECT_EQ(s.step(), 1999ms);
  EXPECT_EQ(s.step(), 3999ms);
  EXPECT_EQ(s.step(), 7999ms);
  EXPECT_EQ(backend.tokens_used.size(), 0u);
}

TEST(Supervisor, RelogsInAfterUpgradeRejected) {
  FakeBackend backend;
  backend.logins = {ok("t1"), ok("t2")};
  backend.tunnels = {ended(TunnelEnd::AuthRejected), ended(TunnelEnd::Closed)};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  s.step();
  s.step();
  EXPECT_EQ(backend.login_calls, 2);
  EXPECT_EQ(backend.tokens_used, (std::vector<std::string>{"t1", "t2"}));
}

TEST(Supervisor, QuickDropsKeepBackingOff) {
  FakeBackend backend;
  backend.logins = {ok("a"), ok("b"), ok("c")};
  backend.tunnels = {ended(TunnelEnd::Closed, 5s), ended(TunnelEnd::Dead, 10s), ended(TunnelEnd::ConnectFailed)};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  EXPECT_EQ(s.step(), 999ms);
  EXPECT_EQ(s.step(), 1999ms);
  EXPECT_EQ(s.step(), 3999ms);
}

TEST(Supervisor, StableTunnelResetsBackoff) {
  FakeBackend backend;
  backend.logins = {failed(), failed(), failed(), ok("a")};
  backend.tunnels = {ended(TunnelEnd::Closed, 2h)};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  s.step();
  s.step();
  s.step();
  EXPECT_EQ(s.step(), 999ms);  // back to the base delay after hours of uptime
}

TEST(Supervisor, TokenExpiryReconnectsImmediately) {
  FakeBackend backend;
  backend.logins = {failed(), ok("a"), ok("b")};
  backend.tunnels = {ended(TunnelEnd::TokenExpired, 30min), ended(TunnelEnd::Closed, 1s)};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  s.step();
  EXPECT_EQ(s.step(), 0ms);
  EXPECT_EQ(s.step(), 999ms);  // and the backoff was reset by it
}

TEST(Supervisor, CredentialsRotatedReconnectsImmediatelyAndResetsBackoff) {
  FakeBackend backend;
  backend.logins = {failed(), failed(), ok("a"), failed()};
  backend.tunnels = {ended(TunnelEnd::CredentialsRotated, 5s)};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  s.step();
  s.step();
  EXPECT_EQ(s.step(), 0ms);
  EXPECT_EQ(s.step(), 999ms);
}

TEST(Supervisor, RunStopsWhenAsked) {
  FakeBackend backend;
  backend.logins = {failed(), failed(), failed()};
  auto backoff = exact_backoff();
  Supervisor s(backend, backoff);

  int sleeps = 0;
  s.run([&] { return sleeps >= 2; }, [&](ms) { ++sleeps; });
  EXPECT_EQ(backend.login_calls, 2);
}
