#include "backoff.h"

#include <gtest/gtest.h>

using namespace bloom_agent;
using ms = std::chrono::milliseconds;

TEST(Backoff, CeilingDoublesUpToCap) {
  Backoff b(ms(1000), ms(60000), [] { return 0.999; });
  std::vector<ms::rep> ceilings;
  for (int i = 0; i < 9; ++i) {
    ceilings.push_back(b.ceiling().count());
    b.next();
  }
  EXPECT_EQ(ceilings, (std::vector<ms::rep>{1000, 2000, 4000, 8000, 16000, 32000, 60000, 60000, 60000}));
}

TEST(Backoff, DelayIsJitteredWithinFloorAndCeiling) {
  double r = 0.0;
  Backoff b(ms(1000), ms(60000), [&] { return r; });
  for (int i = 0; i < 6; ++i) b.next();
  r = 0.0;
  EXPECT_EQ(b.next().count(), 250);  // floor, never a hot loop
  r = 0.5;
  EXPECT_EQ(b.next().count(), 30000);
  r = 0.999999;
  EXPECT_LE(b.next().count(), 60000);
}

TEST(Backoff, ResetStartsOver) {
  Backoff b(ms(1000), ms(60000), [] { return 0.5; });
  for (int i = 0; i < 5; ++i) b.next();
  b.reset();
  EXPECT_EQ(b.attempts(), 0);
  EXPECT_EQ(b.next().count(), 500);
}

TEST(Backoff, NeverGivesUpOrOverflows) {
  Backoff b(ms(1000), ms(60000), [] { return 0.999; });
  for (int i = 0; i < 10000; ++i) {
    const auto d = b.next();
    ASSERT_GE(d.count(), 250);
    ASSERT_LE(d.count(), 60000);
  }
}
