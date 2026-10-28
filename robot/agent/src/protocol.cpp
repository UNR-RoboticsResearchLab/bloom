#include "protocol.h"

#include <nlohmann/json.hpp>

namespace bloom_agent {
namespace {
int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
}  // namespace

std::optional<Uuid> parse_uuid(std::string_view text) {
  if (text.size() != 36) return std::nullopt;
  Uuid id{};
  size_t byte = 0;
  for (size_t i = 0; i < text.size();) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (text[i] != '-') return std::nullopt;
      ++i;
      continue;
    }
    const int hi = hex_value(text[i]), lo = hex_value(text[i + 1]);
    if (hi < 0 || lo < 0) return std::nullopt;
    id[byte++] = static_cast<std::uint8_t>(hi << 4 | lo);
    i += 2;
  }
  return id;
}

std::string format_uuid(const Uuid& id) {
  static constexpr char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(36);
  for (size_t i = 0; i < id.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) out.push_back('-');
    out.push_back(digits[id[i] >> 4]);
    out.push_back(digits[id[i] & 0xf]);
  }
  return out;
}

std::string encode_data_frame(const Uuid& session, std::string_view payload) {
  std::string frame;
  frame.reserve(kFrameHeaderLength + payload.size());
  frame.push_back(static_cast<char>(kDataFrame));
  frame.append(reinterpret_cast<const char*>(session.data()), session.size());
  frame.append(payload);
  return frame;
}

bool decode_data_frame(std::string_view frame, Uuid& session, std::string_view& payload) {
  if (frame.size() < kFrameHeaderLength || static_cast<std::uint8_t>(frame[0]) != kDataFrame) return false;
  for (size_t i = 0; i < session.size(); ++i) session[i] = static_cast<std::uint8_t>(frame[1 + i]);
  payload = frame.substr(kFrameHeaderLength);
  return true;
}

std::string hello_message(std::string_view version, std::string_view commit) {
  return nlohmann::json{{"type", "hello"}, {"version", version}, {"commit", commit}}.dump();
}

std::string ping_message() { return R"({"type":"ping"})"; }

std::string exited_message(const Uuid& session, std::optional<int> code) {
  nlohmann::json msg{{"type", "exited"}, {"sessionId", format_uuid(session)}};
  msg["code"] = code ? nlohmann::json(*code) : nlohmann::json(nullptr);
  return msg.dump();
}

}  // namespace bloom_agent
