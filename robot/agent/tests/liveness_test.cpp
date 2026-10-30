#include "liveness.h"

#include <gtest/gtest.h>

using namespace bloom_agent;
using namespace std::chrono_literals;

TEST(Liveness, PingsOnInterval) {
  const auto t0 = Liveness::clock::time_point{};
  Liveness l(25s, 60s, t0);
  EXPECT_FALSE(l.ping_due(t0 + 24s));
  EXPECT_TRUE(l.ping_due(t0 + 25s));
  l.on_ping_sent(t0 + 25s);
  EXPECT_FALSE(l.ping_due(t0 + 49s));
  EXPECT_TRUE(l.ping_due(t0 + 50s));
}

TEST(Liveness, DeadAfterSilence_EvenWhilePinging) {
  const auto t0 = Liveness::clock::time_point{};
  Liveness l(25s, 60s, t0);
  l.on_ping_sent(t0 + 25s);
  l.on_ping_sent(t0 + 50s);  // pings going out don't count -- only what comes back does
  EXPECT_FALSE(l.is_dead(t0 + 59s));
  EXPECT_TRUE(l.is_dead(t0 + 60s));
}

TEST(Liveness, AnyTrafficKeepsItAlive) {
  const auto t0 = Liveness::clock::time_point{};
  Liveness l(25s, 60s, t0);
  l.on_received(t0 + 50s);
  EXPECT_FALSE(l.is_dead(t0 + 100s));
  EXPECT_TRUE(l.is_dead(t0 + 110s));
}
