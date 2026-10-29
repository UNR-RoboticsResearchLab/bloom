#include "ota_manifest.h"

#include <gtest/gtest.h>

#include "ota_test_util.h"

using namespace bloom_agent::ota;
using ota_test::signed_manifest;
using ota_test::TestKey;

namespace {
const TestKey& key() {
  static TestKey k;
  return k;
}
Policy policy() { return {{"jazzy-stable", "jazzy-beta"}, key().public_pem}; }
const auto kNow = std::chrono::system_clock::from_time_t(1'800'000'000);  // 2027-01-15

std::string check(const nlohmann::json& json, const Policy& p = policy()) {
  std::string error, id;
  const auto m = parse_manifest(json, error, id);
  if (!m) return "parse: " + error;
  return validate(*m, p, kNow);
}
}  // namespace

TEST(OtaManifest, AcceptsAWellFormedSignedManifest) {
  EXPECT_EQ(check(signed_manifest(key(), "job-1", "apply")), "");
  EXPECT_EQ(check(signed_manifest(key(), "job-2", "stage")), "");
}

TEST(OtaManifest, RejectsAnExpiredManifest) {
  EXPECT_EQ(check(signed_manifest(key(), "j", "stage", "1.0.0", "jazzy-stable", "2027-01-01T00:00:00Z")), "manifest has expired");
}

TEST(OtaManifest, ExpiryIsExclusiveOfTheInstant) {
  // kNow is 2027-01-15T08:00:00Z.
  EXPECT_EQ(check(signed_manifest(key(), "j", "stage", "1.0.0", "jazzy-stable", "2027-01-15T08:00:00Z")), "manifest has expired");
  EXPECT_EQ(check(signed_manifest(key(), "j", "stage", "1.0.0", "jazzy-stable", "2027-01-15T08:00:01Z")), "");
}

TEST(OtaManifest, RejectsAnUnparseableExpiry) {
  for (const char* bad : {"tomorrow", "2999-01-01", "2999-02-30T00:00:00Z", "2999-01-01T00:00:00+01:00", ""}) {
    const auto reason = check(signed_manifest(key(), "j", "stage", "1.0.0", "jazzy-stable", bad));
    EXPECT_NE(reason.find("expiresAt"), std::string::npos) << bad << " -> " << reason;
  }
}

TEST(OtaManifest, RejectsABadSignature) {
  auto tampered_field = signed_manifest(key(), "j", "stage");
  tampered_field["version"] = "9.9.9";  // signed for 1.4.2-1jazzy
  EXPECT_EQ(check(tampered_field), "manifest signature is invalid");

  auto tampered_action = signed_manifest(key(), "j", "stage");
  tampered_action["action"] = "apply";  // a signed stage must not be replayable as an apply
  EXPECT_EQ(check(tampered_action), "manifest signature is invalid");

  auto garbage = signed_manifest(key(), "j", "stage");
  garbage["signature"] = "AAAA";
  EXPECT_EQ(check(garbage), "manifest signature is invalid");

  static const TestKey other;
  EXPECT_EQ(check(signed_manifest(other, "j", "stage")), "manifest signature is invalid");
}

TEST(OtaManifest, RejectsEverythingWithoutAConfiguredKey) {
  Policy p = policy();
  p.public_key_pem.clear();
  EXPECT_NE(check(signed_manifest(key(), "j", "stage"), p).find("no manifest signing key"), std::string::npos);
  p.public_key_pem = "not a pem";
  EXPECT_EQ(check(signed_manifest(key(), "j", "stage"), p), "manifest signature is invalid");
}

TEST(OtaManifest, RejectsADisallowedSuite) {
  EXPECT_EQ(check(signed_manifest(key(), "j", "stage", "1.0.0", "jazzy-evil")), "suite is not allowed on this robot");
  Policy none = policy();
  none.allowed_suites.clear();
  EXPECT_EQ(check(signed_manifest(key(), "j", "stage"), none), "suite is not allowed on this robot");
}

TEST(OtaManifest, RejectsAnUnsupportedSourceType) {
  auto m = signed_manifest(key(), "j", "stage");
  m["source"]["type"] = "http";
  EXPECT_EQ(check(m), "unsupported source type \"http\"");
}

TEST(OtaManifest, RejectsBadVersionStrings) {
  for (const char* bad : {"", "1.0; rm -rf /", "1.0 && reboot", "$(id)", "`id`", "1.0\n2", "-1.0", "v1.0", "1.0|x", "1:2.0", "1.0'x", "1.0\"x",
                          "1.0*", "1.0/../x", "1.0=2", std::string(65, '1').c_str()}) {
    std::string error, id;
    nlohmann::json j = signed_manifest(key(), "j", "stage");
    j["version"] = bad;
    EXPECT_FALSE(parse_manifest(j, error, id)) << "accepted: " << bad;
  }
  for (const char* good : {"1.4.2", "1.4.2-1jazzy", "2.0~rc1", "1.0+git20260101.abc", "10.0.0-0ubuntu1"})
    EXPECT_TRUE(valid_version(good)) << good;
}

TEST(OtaManifest, RejectsBadJobIds) {
  for (const char* bad : {"", "a b", "a/b", "a;b", "../x", "a.service", "a$b", "a\nb"}) {
    std::string error, id;
    nlohmann::json j = signed_manifest(key(), "j", "stage");
    j["jobId"] = bad;
    EXPECT_FALSE(parse_manifest(j, error, id)) << bad;
    EXPECT_EQ(id, "") << "an invalid id must not be echoed back";
  }
}

TEST(OtaManifest, ParseKeepsTheJobIdForRejectionReports) {
  std::string error, id;
  nlohmann::json j = signed_manifest(key(), "job-9", "stage");
  j["action"] = "reboot";
  EXPECT_FALSE(parse_manifest(j, error, id));
  EXPECT_EQ(id, "job-9");
}

TEST(OtaManifest, RejectsMalformedShapes) {
  std::string error, id;
  EXPECT_FALSE(parse_manifest(nlohmann::json::array(), error, id));
  EXPECT_FALSE(parse_manifest("text", error, id));
  for (const char* drop : {"jobId", "action", "version", "source", "signature", "expiresAt"}) {
    auto j = signed_manifest(key(), "j", "stage");
    j.erase(drop);
    EXPECT_FALSE(parse_manifest(j, error, id)) << drop;
  }
  auto j = signed_manifest(key(), "j", "stage");
  j["sha256"] = "nothex";
  EXPECT_FALSE(parse_manifest(j, error, id));
  j["sha256"] = std::string(64, 'a');
  EXPECT_TRUE(parse_manifest(j, error, id));
}

TEST(OtaManifest, TimestampParsing) {
  EXPECT_EQ(parse_utc_timestamp("1970-01-01T00:00:00Z"), 0);
  EXPECT_EQ(parse_utc_timestamp("2000-03-01T00:00:00Z"), 951868800);
  EXPECT_FALSE(parse_utc_timestamp("2000-13-01T00:00:00Z"));
  EXPECT_FALSE(parse_utc_timestamp("2000-01-01 00:00:00Z"));
}

// A vector made with the openssl CLI, exactly as docs/ota.md tells operators to sign: proves the
// documented recipe and the agent's canonical payload agree.
TEST(OtaManifest, VerifiesTheDocumentedOpensslSignature) {
  const std::string pub =
      "-----BEGIN PUBLIC KEY-----\nMCowBQYDK2VwAyEADWmZm953S7wr9kzqbm9Suvx86XVMDNtdDyQH5s+f9IY=\n-----END PUBLIC KEY-----\n";
  const nlohmann::json manifest = {
      {"jobId", "9b2f6c1e-example-0001"}, {"action", "apply"}, {"version", "1.4.2-1jazzy"},
      {"source", {{"type", "apt"}, {"suite", "jazzy-stable"}}},
      {"signature", "r/MpwXjJJHlQx+DRZwGhqct144KqtDAY6VGNyqVk17fAO0lU/gZSXj1q9HFEsZEowsHB/VECAWRpNmW10YjaAA=="},
      {"expiresAt", "2999-01-01T00:00:00Z"}};
  std::string error, id;
  const auto m = parse_manifest(manifest, error, id);
  ASSERT_TRUE(m) << error;
  EXPECT_EQ(validate(*m, {{"jazzy-stable"}, pub}, kNow), "");
}
