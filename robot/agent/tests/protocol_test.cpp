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
