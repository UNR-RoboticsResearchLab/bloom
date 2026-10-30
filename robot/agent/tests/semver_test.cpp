#include "semver.h"

#include <gtest/gtest.h>

using namespace bloom_agent;

namespace {
SemVer v(const char* s) {
  auto parsed = SemVer::parse(s);
  if (!parsed) throw std::invalid_argument(s);
  return *parsed;
}
}  // namespace

TEST(SemVer, ComparesNumericallyNotLexically) {
  EXPECT_GT(v("1.0.10"), v("1.0.9"));
  EXPECT_GT(v("1.10.0"), v("1.9.99"));
  EXPECT_GT(v("2.0.0"), v("1.99.99"));
}

TEST(SemVer, PrereleaseSortsBeforeRelease) {
  EXPECT_LT(v("1.0.5-g1a2b3c4d"), v("1.0.5"));  // NBGV non-main build of the same height
  EXPECT_GT(v("1.0.6-g1a2b3c4d"), v("1.0.5"));
}

TEST(SemVer, PrereleasePrecedenceFollowsSpec) {
  // semver.org section 11 example chain.
  const char* chain[] = {"1.0.0-alpha", "1.0.0-alpha.1", "1.0.0-alpha.beta", "1.0.0-beta",
                         "1.0.0-beta.2", "1.0.0-beta.11", "1.0.0-rc.1", "1.0.0"};
  for (size_t i = 0; i + 1 < std::size(chain); ++i) EXPECT_LT(v(chain[i]), v(chain[i + 1])) << chain[i];
}

TEST(SemVer, BuildMetadataIsIgnored) {
  EXPECT_EQ(v("0.0.0-local+1a2b3c"), v("0.0.0-local+ffffff"));
  EXPECT_EQ(v("1.2.3+abc").build, "abc");
}

TEST(SemVer, RejectsInvalid) {
  for (const char* bad : {"", "1", "1.0", "1.0.0.0", "01.0.0", "1.0.0-", "1.0.0-01", "1.0.0+", "v1.0.0", "1.0.0-a..b"})
    EXPECT_FALSE(SemVer::parse(bad)) << bad;
}
