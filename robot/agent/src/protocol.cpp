#include "protocol.h"

#include <cctype>

#include "base64.h"
#include "signalr.h"

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

std::string hello_message(std::string_view version, std::string_view commit) {
  return signalr::invocation("Hello", nlohmann::json::array({version, commit}));
}

std::string ota_progress_message(std::string_view job_id, std::string_view state, int percent, std::string_view detail) {
  return signalr::invocation("ReportOtaProgress", nlohmann::json::array({job_id, state, percent, detail}));
}

std::string installed_version_message(std::string_view version) {
  return signalr::invocation("ReportInstalledVersion", nlohmann::json::array({version}));
}

std::string output_message(const Uuid& session, std::string_view bytes) {
  return signalr::invocation("Output", nlohmann::json::array({format_uuid(session), base64_encode(bytes)}));
}

std::string exited_message(const Uuid& session, std::optional<int> code) {
  return signalr::invocation(
      "SessionExited", nlohmann::json::array({format_uuid(session), code ? nlohmann::json(*code) : nlohmann::json(nullptr)}));
}

std::string secret_applied_message(std::string_view request_id, bool ok, std::string_view message) {
  return signalr::invocation("SecretApplied",
                             nlohmann::json::array({request_id, ok, message.empty() ? nlohmann::json(nullptr) : nlohmann::json(message)}));
}

std::optional<std::string> arg_string(const nlohmann::json& args, std::size_t index) {
  if (!args.is_array() || index >= args.size() || !args[index].is_string()) return std::nullopt;
  return args[index].get<std::string>();
}

std::optional<Uuid> arg_uuid(const nlohmann::json& args, std::size_t index) {
  const auto text = arg_string(args, index);
  return text ? parse_uuid(*text) : std::nullopt;
}

std::optional<int> arg_int(const nlohmann::json& args, std::size_t index) {
  if (!args.is_array() || index >= args.size() || !args[index].is_number_integer()) return std::nullopt;
  return args[index].get<int>();
}

std::optional<std::string> arg_bytes(const nlohmann::json& args, std::size_t index) {
  const auto text = arg_string(args, index);
  return text ? base64_decode(*text) : std::nullopt;
}

std::optional<std::string> request_id_of(const nlohmann::json& args) {
  const auto id = arg_uuid(args, 0);
  return id ? std::optional<std::string>(format_uuid(*id)) : std::nullopt;
}

std::optional<SetSecretRequest> parse_set_secret(const nlohmann::json& args) {
  auto id = request_id_of(args);
  auto secret = arg_string(args, 1);
  if (!id || !secret || secret->size() < 16 || secret->size() > 512) return std::nullopt;
  for (char c : *secret)
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_') return std::nullopt;
  return SetSecretRequest{*id, std::move(*secret)};
}

}  // namespace bloom_agent
