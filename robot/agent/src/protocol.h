// Wire format of the agent <-> backend tunnel. Keep in sync with
// backend/Services/RemoteShell/ShellProtocol.cs.
//
// Control messages: JSON text frames {"type": "..."}.
//   agent -> backend: hello{version,commit}, ping, exited{sessionId,code}
//   agent -> backend: also secret_applied{requestId,ok,message}
//   backend -> agent: open{sessionId,cols,rows}, resize{sessionId,cols,rows}, close{sessionId}, pong,
//                     set_secret{requestId,secret}  (carries a secret: never log it)
// Terminal data: binary frames [0x01][16-byte session id, RFC 4122 byte order][payload].
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace bloom_agent {

using Uuid = std::array<std::uint8_t, 16>;

constexpr std::uint8_t kDataFrame = 0x01;
constexpr std::size_t kFrameHeaderLength = 17;

// Accepts the canonical 8-4-4-4-12 form (what .NET's Guid serializes to).
std::optional<Uuid> parse_uuid(std::string_view text);
std::string format_uuid(const Uuid& id);

std::string encode_data_frame(const Uuid& session, std::string_view payload);
bool decode_data_frame(std::string_view frame, Uuid& session, std::string_view& payload);

std::string hello_message(std::string_view version, std::string_view commit);
std::string ping_message();
std::string exited_message(const Uuid& session, std::optional<int> code);

struct SetSecretRequest {
  std::string request_id;
  std::string secret;
};
// Parses a set_secret message; nullopt unless it has a GUID requestId and a secret made of the
// characters the backend generates (base64url), so nothing odd can reach a shell or a file.
std::optional<SetSecretRequest> parse_set_secret(std::string_view json_text);
// Best-effort requestId of a set_secret message even when it is otherwise invalid, so the backend
// still gets an answer instead of waiting for its timeout.
std::optional<std::string> request_id_of(std::string_view json_text);
std::string secret_applied_message(std::string_view request_id, bool ok, std::string_view message);

}  // namespace bloom_agent
