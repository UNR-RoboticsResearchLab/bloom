#include "protocol.h"

#include "base64.h"

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

using namespace bloom_agent;

TEST(Protocol, UuidRoundTripsInRfc4122ByteOrder) {
  // Same bytes .NET writes with Guid.TryWriteBytes(bigEndian: true).
  const auto id = parse_uuid("00112233-4455-6677-8899-aabbccddeeff");
  ASSERT_TRUE(id);
  EXPECT_EQ((*id)[0], 0x00);
  EXPECT_EQ((*id)[3], 0x33);
  EXPECT_EQ((*id)[15], 0xff);
  EXPECT_EQ(format_uuid(*id), "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_EQ(format_uuid(*parse_uuid("00112233-4455-6677-8899-AABBCCDDEEFF")), "00112233-4455-6677-8899-aabbccddeeff");
}

TEST(Protocol, RejectsMalformedUuids) {
  EXPECT_FALSE(parse_uuid(""));
  EXPECT_FALSE(parse_uuid("00112233445566778899aabbccddeeff"));
  EXPECT_FALSE(parse_uuid("00112233-4455-6677-8899-aabbccddeefg"));
  EXPECT_FALSE(parse_uuid("0011223-34455-6677-8899-aabbccddeeff"));
}

TEST(Protocol, AgentMessagesAreFramedHubInvocations) {
  const auto id = *parse_uuid("00112233-4455-6677-8899-aabbccddeeff");
  const auto parse = [](const std::string& framed) {
    EXPECT_EQ(framed.back(), '\x1e');
    return nlohmann::json::parse(framed.substr(0, framed.size() - 1));
  };

  const auto hello = parse(hello_message("1.0.7", "abc"));
  EXPECT_EQ(hello["target"], "Hello");
  EXPECT_EQ(hello["arguments"], nlohmann::json::array({"1.0.7", "abc"}));

  // Terminal bytes travel as base64 in the JSON protocol, NULs and escape sequences included.
  const auto out = parse(output_message(id, std::string("ls\0\x1b[0m\r\n", 9)));
  EXPECT_EQ(out["target"], "Output");
  EXPECT_EQ(out["arguments"][0], "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_EQ(base64_decode(out["arguments"][1].get<std::string>()), std::string("ls\0\x1b[0m\r\n", 9));

  EXPECT_EQ(parse(exited_message(id, 3))["arguments"][1], 3);
  EXPECT_TRUE(parse(exited_message(id, std::nullopt))["arguments"][1].is_null());
}

TEST(Protocol, SecretAppliedMessage) {
  const auto parse = [](const std::string& framed) { return nlohmann::json::parse(framed.substr(0, framed.size() - 1)); };
  const auto ok = parse(secret_applied_message("id-1", true, ""));
  EXPECT_EQ(ok["target"], "SecretApplied");
  EXPECT_EQ(ok["arguments"][0], "id-1");
  EXPECT_TRUE(ok["arguments"][1].get<bool>());
  EXPECT_TRUE(ok["arguments"][2].is_null());
  const auto bad = parse(secret_applied_message("id-1", false, "boom"));
  EXPECT_FALSE(bad["arguments"][1].get<bool>());
  EXPECT_EQ(bad["arguments"][2], "boom");
}

TEST(Protocol, ReadsServerArguments) {
  const auto args = nlohmann::json::parse(R"(["00112233-4455-6677-8899-aabbccddeeff",120,"bHM=","text","%%%%"])");
  EXPECT_EQ(arg_uuid(args, 0), parse_uuid("00112233-4455-6677-8899-aabbccddeeff"));
  EXPECT_EQ(arg_int(args, 1), 120);
  EXPECT_EQ(arg_bytes(args, 2), "ls");
  EXPECT_EQ(arg_string(args, 3), "text");
  // Wrong type, bad value, or out of range: nullopt, never a throw.
  EXPECT_FALSE(arg_uuid(args, 1));
  EXPECT_FALSE(arg_int(args, 0));
  EXPECT_FALSE(arg_bytes(args, 4));  // not base64
  EXPECT_FALSE(arg_bytes(args, 1));  // not a string
  EXPECT_FALSE(arg_string(args, 9));
  EXPECT_FALSE(arg_string(nlohmann::json::object(), 0));
}

TEST(Protocol, ParsesSetSecret) {
  const auto args = nlohmann::json::parse(R"(["00112233-4455-6677-8899-aabbccddeeff","Xq3_-abcDEF0123456789"])");
  const auto req = parse_set_secret(args);
  ASSERT_TRUE(req);
  EXPECT_EQ(req->request_id, "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_EQ(req->secret, "Xq3_-abcDEF0123456789");
}

TEST(Protocol, RejectsUnsafeOrMalformedSecrets) {
  const auto with = [](const std::string& secret) {
    return nlohmann::json::array({"00112233-4455-6677-8899-aabbccddeeff", secret});
  };
  EXPECT_TRUE(parse_set_secret(with("abcdefghijklmnop")));
  EXPECT_FALSE(parse_set_secret(with("short")));
  EXPECT_FALSE(parse_set_secret(with(std::string(513, 'a'))));
  EXPECT_FALSE(parse_set_secret(with("abcdefghijklmnop\nBLOOM=1")));  // nothing that could reach a shell or a file
  EXPECT_FALSE(parse_set_secret(with("abcdefghijklmn'; rm -rf /")));
  EXPECT_FALSE(parse_set_secret(nlohmann::json::array({"nope", "abcdefghijklmnop"})));
  EXPECT_FALSE(parse_set_secret(nlohmann::json::array({"00112233-4455-6677-8899-aabbccddeeff", 42})));
  EXPECT_FALSE(parse_set_secret(nlohmann::json::object()));
}

TEST(Protocol, RequestIdIsRecoverableFromInvalidSecrets) {
  // So the backend still gets an answer rather than waiting out its timeout.
  const auto args = nlohmann::json::array({"00112233-4455-6677-8899-aabbccddeeff", "bad!"});
  EXPECT_FALSE(parse_set_secret(args));
  EXPECT_EQ(request_id_of(args), "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_FALSE(request_id_of(nlohmann::json::array({"x"})));
}

TEST(OtaProtocol, ReportMessagesUseTheHubMethodNames) {
  const auto progress = bloom_agent::ota_progress_message("job-1", "staging", 40, "downloading");
  EXPECT_NE(progress.find("\"target\":\"ReportOtaProgress\""), std::string::npos);
  EXPECT_NE(progress.find("[\"job-1\",\"staging\",40,\"downloading\"]"), std::string::npos);
  EXPECT_EQ(progress.back(), '\x1e');
  const auto version = bloom_agent::installed_version_message("1.4.2-1jazzy");
  EXPECT_NE(version.find("\"target\":\"ReportInstalledVersion\""), std::string::npos);
  EXPECT_NE(version.find("[\"1.4.2-1jazzy\"]"), std::string::npos);
}
