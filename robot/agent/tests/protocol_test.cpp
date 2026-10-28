#include "protocol.h"

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

TEST(Protocol, DataFrameRoundTrips) {
  const auto id = *parse_uuid("00112233-4455-6677-8899-aabbccddeeff");
  const auto frame = encode_data_frame(id, std::string("ls\0-la\n", 7));
  ASSERT_EQ(frame.size(), 17u + 7u);
  EXPECT_EQ(static_cast<unsigned char>(frame[0]), 0x01);

  Uuid decoded{};
  std::string_view payload;
  ASSERT_TRUE(decode_data_frame(frame, decoded, payload));
  EXPECT_EQ(decoded, id);
  EXPECT_EQ(payload, std::string_view("ls\0-la\n", 7));
}

TEST(Protocol, ParsesSetSecret) {
  const std::string msg = R"({"type":"set_secret","requestId":"00112233-4455-6677-8899-aabbccddeeff","secret":"Xq3_-abcDEF0123456789"})";
  const auto req = parse_set_secret(msg);
  ASSERT_TRUE(req);
  EXPECT_EQ(req->request_id, "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_EQ(req->secret, "Xq3_-abcDEF0123456789");
}

TEST(Protocol, RejectsUnsafeOrMalformedSecrets) {
  const auto with = [](const std::string& secret) {
    return nlohmann::json{{"type", "set_secret"}, {"requestId", "00112233-4455-6677-8899-aabbccddeeff"}, {"secret", secret}}.dump();
  };
  EXPECT_TRUE(parse_set_secret(with("abcdefghijklmnop")));
  EXPECT_FALSE(parse_set_secret(with("short")));
  EXPECT_FALSE(parse_set_secret(with(std::string(513, 'a'))));
  EXPECT_FALSE(parse_set_secret(with("abcdefghijklmnop\nBLOOM=1")));  // nothing that could reach a shell or a file
  EXPECT_FALSE(parse_set_secret(with("abcdefghijklmn'; rm -rf /")));
  EXPECT_FALSE(parse_set_secret(R"({"type":"set_secret","requestId":"nope","secret":"abcdefghijklmnop"})"));
  EXPECT_FALSE(parse_set_secret(R"({"type":"other","requestId":"00112233-4455-6677-8899-aabbccddeeff","secret":"abcdefghijklmnop"})"));
  EXPECT_FALSE(parse_set_secret("not json"));
}

TEST(Protocol, RequestIdIsRecoverableFromInvalidSecrets) {
  // So the backend still gets an answer rather than waiting out its timeout.
  const std::string msg = R"({"type":"set_secret","requestId":"00112233-4455-6677-8899-aabbccddeeff","secret":"bad!"})";
  EXPECT_FALSE(parse_set_secret(msg));
  EXPECT_EQ(request_id_of(msg), "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_FALSE(request_id_of(R"({"requestId":"x"})"));
}

TEST(Protocol, SecretAppliedMessage) {
  const auto ok = nlohmann::json::parse(secret_applied_message("id-1", true, ""));
  EXPECT_EQ(ok["type"], "secret_applied");
  EXPECT_TRUE(ok["ok"].get<bool>());
  EXPECT_FALSE(ok.contains("message"));
  const auto bad = nlohmann::json::parse(secret_applied_message("id-1", false, "boom"));
  EXPECT_FALSE(bad["ok"].get<bool>());
  EXPECT_EQ(bad["message"], "boom");
}

TEST(Protocol, RejectsShortOrUnknownFrames) {
  Uuid id{};
  std::string_view payload;
  EXPECT_FALSE(decode_data_frame(std::string(16, '\x01'), id, payload));
  EXPECT_FALSE(decode_data_frame(std::string(17, '\x02'), id, payload));
  EXPECT_TRUE(decode_data_frame(std::string(17, '\x01'), id, payload));
  EXPECT_TRUE(payload.empty());
}

TEST(Protocol, ControlMessages) {
  const auto hello = nlohmann::json::parse(hello_message("1.0.7", "abc"));
  EXPECT_EQ(hello["type"], "hello");
  EXPECT_EQ(hello["version"], "1.0.7");
  const auto id = *parse_uuid("00112233-4455-6677-8899-aabbccddeeff");
  const auto exited = nlohmann::json::parse(exited_message(id, 0));
  EXPECT_EQ(exited["sessionId"], "00112233-4455-6677-8899-aabbccddeeff");
  EXPECT_EQ(exited["code"], 0);
  EXPECT_TRUE(nlohmann::json::parse(exited_message(id, std::nullopt))["code"].is_null());
}
