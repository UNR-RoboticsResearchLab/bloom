#include "signalr.h"

namespace bloom_agent::signalr {

std::string handshake_request() { return std::string(R"({"protocol":"json","version":1})") + kRecordSeparator; }

HandshakeResponse parse_handshake_response(std::string_view record) {
  const auto json = nlohmann::json::parse(record, nullptr, /*allow_exceptions=*/false);
  if (!json.is_object()) return {false, "the server's handshake reply was not a JSON object"};
  if (json.contains("error") && json["error"].is_string()) return {false, json["error"].get<std::string>()};
  return {true, ""};
}

std::string invocation(std::string_view target, const nlohmann::json& arguments) {
  nlohmann::json msg{{"type", 1}, {"target", target}, {"arguments", arguments}};
  return msg.dump() + kRecordSeparator;
}

std::string ping() { return std::string(R"({"type":6})") + kRecordSeparator; }

std::optional<std::string> RecordParser::next() {
  const auto end = buffer_.find(kRecordSeparator);
  if (end == std::string::npos) return std::nullopt;
  std::string record = buffer_.substr(0, end);
  buffer_.erase(0, end + 1);
  return record;
}

std::optional<Message> parse_message(std::string_view record) {
  const auto json = nlohmann::json::parse(record, nullptr, /*allow_exceptions=*/false);
  if (!json.is_object() || !json.contains("type") || !json["type"].is_number_integer()) return std::nullopt;

  Message msg;
  const int type = json["type"].get<int>();
  msg.type = type >= 1 && type <= 7 ? static_cast<MessageType>(type) : MessageType::Unknown;
  if (json.contains("target") && json["target"].is_string()) msg.target = json["target"].get<std::string>();
  msg.arguments = json.contains("arguments") && json["arguments"].is_array() ? json["arguments"] : nlohmann::json::array();
  if (json.contains("error") && json["error"].is_string()) msg.error = json["error"].get<std::string>();
  return msg;
}

}  // namespace bloom_agent::signalr
