#include "base64.h"

#include <gtest/gtest.h>

using namespace bloom_agent;

TEST(Base64, RfcVectors) {
  EXPECT_EQ(base64_encode(""), "");
  EXPECT_EQ(base64_encode("f"), "Zg==");
  EXPECT_EQ(base64_encode("fo"), "Zm8=");
  EXPECT_EQ(base64_encode("foo"), "Zm9v");
  EXPECT_EQ(base64_encode("foob"), "Zm9vYg==");
  EXPECT_EQ(base64_encode("fooba"), "Zm9vYmE=");
  EXPECT_EQ(base64_encode("foobar"), "Zm9vYmFy");
  EXPECT_EQ(base64_encode("user:pass"), "dXNlcjpwYXNz");
}

TEST(Base64, DecodesTheSameVectors) {
  EXPECT_EQ(base64_decode(""), "");
  EXPECT_EQ(base64_decode("Zg=="), "f");
  EXPECT_EQ(base64_decode("Zm8="), "fo");
  EXPECT_EQ(base64_decode("Zm9vYmFy"), "foobar");
}

TEST(Base64, RoundTripsEveryByteValue) {
  // Terminal output is arbitrary bytes: escape sequences, NULs, invalid UTF-8.
  std::string all;
  for (int i = 0; i < 256; ++i) all.push_back(static_cast<char>(i));
  for (size_t len : {size_t{0}, size_t{1}, size_t{2}, size_t{3}, size_t{255}, size_t{256}}) {
    const auto text = all.substr(0, len);
    EXPECT_EQ(base64_decode(base64_encode(text)), text) << len;
  }
  const std::string with_nul("a\0b\r\n\x1b[0m", 9);
  EXPECT_EQ(base64_decode(base64_encode(with_nul)), with_nul);
}

TEST(Base64, RejectsMalformedInput) {
  for (const char* bad : {"Zg=", "Zg", "Z===", "====", "Zg==Zg==", "Zm9v!mFy", "Zm 9v", "Zg=a", "=Zg="})
    EXPECT_FALSE(base64_decode(bad)) << bad;
}
