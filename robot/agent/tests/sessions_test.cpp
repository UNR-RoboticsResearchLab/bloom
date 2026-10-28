#include "sessions.h"

#include <gtest/gtest.h>

using namespace bloom_agent;

namespace {
struct FakePty : IPty {
  struct Log {
    int terminated = 0;
    int last_cols = 0, last_rows = 0;
  };
  explicit FakePty(std::shared_ptr<Log> log) : log(std::move(log)) {}
  int fd() const override { return -1; }
  void resize(int cols, int rows) override {
    log->last_cols = cols;
    log->last_rows = rows;
  }
  void terminate() override { ++log->terminated; }
  std::optional<int> wait_exit(std::chrono::milliseconds) override { return 0; }
  std::shared_ptr<Log> log;
};

Uuid id(int n) {
  Uuid u{};
  u[15] = static_cast<std::uint8_t>(n);
  return u;
}
}  // namespace

TEST(Sessions, OpenResizeClose) {
  auto log = std::make_shared<FakePty::Log>();
  SessionManager sm([&](int cols, int rows) {
    log->last_cols = cols;
    log->last_rows = rows;
    return std::make_unique<FakePty>(log);
  });

  ASSERT_NE(sm.open(id(1), 80, 24), nullptr);
  EXPECT_EQ(log->last_cols, 80);
  EXPECT_TRUE(sm.resize(id(1), 120, 40));
  EXPECT_EQ(log->last_cols, 120);
  EXPECT_EQ(log->last_rows, 40);
  EXPECT_TRUE(sm.close(id(1)));
  EXPECT_EQ(log->terminated, 1);
  EXPECT_EQ(sm.size(), 0u);
  EXPECT_FALSE(sm.close(id(1)));
}

TEST(Sessions, RejectsDuplicateIdsAndEnforcesLimit) {
  auto log = std::make_shared<FakePty::Log>();
  SessionManager sm([&](int, int) { return std::make_unique<FakePty>(log); });
  ASSERT_NE(sm.open(id(1), 80, 24), nullptr);
  EXPECT_EQ(sm.open(id(1), 80, 24), nullptr);
  for (int i = 2; i <= static_cast<int>(SessionManager::kMaxSessions); ++i) ASSERT_NE(sm.open(id(i), 80, 24), nullptr);
  EXPECT_EQ(sm.open(id(99), 80, 24), nullptr);
}

TEST(Sessions, CloseAllTerminatesEveryShell) {
  auto log = std::make_shared<FakePty::Log>();
  SessionManager sm([&](int, int) { return std::make_unique<FakePty>(log); });
  sm.open(id(1), 80, 24);
  sm.open(id(2), 80, 24);
  sm.close_all();
  EXPECT_EQ(log->terminated, 2);
  EXPECT_EQ(sm.size(), 0u);
}

TEST(Sessions, FactoryFailureIsContained) {
  SessionManager sm([](int, int) -> std::unique_ptr<IPty> { throw std::runtime_error("no pty"); });
  EXPECT_EQ(sm.open(id(1), 80, 24), nullptr);
  EXPECT_EQ(sm.size(), 0u);
}
