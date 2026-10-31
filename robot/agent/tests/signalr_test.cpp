#include "signalr.h"

#include <gtest/gtest.h>

using namespace bloom_agent;
using nlohmann::json;
namespace sr = bloom_agent::signalr;

namespace {
std::string rs(const std::string& s) { return s + sr::kRecordSeparator; }
}  // namespace

TEST(SignalR, HandshakeRequestIsJsonProtocolV1TerminatedByRecordSeparator) {
  EXPECT_EQ(sr::handshake_request(), "{\"protocol\":\"json\",\"version\":1}\x1e");
}

TEST(SignalR, HandshakeResponse) {
  EXPECT_TRUE(sr::parse_handshake_response("{}").ok);
  const auto refused = sr::parse_handshake_response(R"({"error":"The protocol 'x' is not supported."})");
  EXPECT_FALSE(refused.ok);
  EXPECT_EQ(refused.error, "The protocol 'x' is not supported.");
  EXPECT_FALSE(sr::parse_handshake_response("<html>captive portal</html>").ok);
}

TEST(SignalR, InvocationIsFireAndForget) {
  const auto framed = sr::invocation("Hello", json::array({"1.0.7", "abc"}));
  ASSERT_EQ(framed.back(), sr::kRecordSeparator);
  const auto msg = json::parse(framed.substr(0, framed.size() - 1));
  EXPECT_EQ(msg["type"], 1);
  EXPECT_EQ(msg["target"], "Hello");
  EXPECT_EQ(msg["arguments"], json::array({"1.0.7", "abc"}));
  EXPECT_FALSE(msg.contains("invocationId"));  // no invocationId => the server sends no completion
}

TEST(SignalR, PingIsType6) { EXPECT_EQ(sr::ping(), "{\"type\":6}\x1e"); }

TEST(SignalR, ParserSplitsSeveralRecordsInOneMessage) {
  sr::RecordParser p;
  p.feed(rs("{\"type\":6}") + rs("{\"type\":1,\"target\":\"Open\",\"arguments\":[]}"));
  EXPECT_EQ(p.next(), "{\"type\":6}");
  EXPECT_EQ(p.next(), "{\"type\":1,\"target\":\"Open\",\"arguments\":[]}");
  EXPECT_FALSE(p.next());
}

TEST(SignalR, ParserHoldsAPartialRecordUntilItsSeparatorArrives) {
  sr::RecordParser p;
  p.feed("{\"type\":");
  EXPECT_FALSE(p.next());
  p.feed("6}");
  EXPECT_FALSE(p.next());
  p.feed(std::string(1, sr::kRecordSeparator) + "{\"ty");
  EXPECT_EQ(p.next(), "{\"type\":6}");
  EXPECT_FALSE(p.next());  // the next record is still incomplete
  p.feed("pe\":7}" + std::string(1, sr::kRecordSeparator));
  EXPECT_EQ(p.next(), "{\"type\":7}");
}

TEST(SignalR, ParsesAnInvocationFromTheServer) {
  const auto msg = sr::parse_message(R"({"type":1,"target":"Resize","arguments":["00112233-4455-6677-8899-aabbccddeeff",120,40]})");
  ASSERT_TRUE(msg);
  EXPECT_EQ(msg->type, sr::MessageType::Invocation);
  EXPECT_EQ(msg->target, "Resize");
  ASSERT_EQ(msg->arguments.size(), 3u);
  EXPECT_EQ(msg->arguments[1], 120);
}

TEST(SignalR, ParsesPingAndClose) {
  EXPECT_EQ(sr::parse_message(R"({"type":6})")->type, sr::MessageType::Ping);
  const auto close = sr::parse_message(R"({"type":7,"error":"Connection closed with an error.","allowReconnect":false})");
  ASSERT_TRUE(close);
  EXPECT_EQ(close->type, sr::MessageType::Close);
  EXPECT_EQ(close->error, "Connection closed with an error.");
}

TEST(SignalR, ToleratesOddMessages) {
  EXPECT_FALSE(sr::parse_message("not json"));
  EXPECT_FALSE(sr::parse_message("[1,2]"));
  EXPECT_FALSE(sr::parse_message(R"({"target":"x"})"));
  EXPECT_EQ(sr::parse_message(R"({"type":99})")->type, sr::MessageType::Unknown);
  // A malformed arguments field becomes an empty array rather than a crash later.
  EXPECT_TRUE(sr::parse_message(R"({"type":1,"target":"x","arguments":"nope"})")->arguments.empty());
}
