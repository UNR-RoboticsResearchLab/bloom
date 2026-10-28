// Wire format of the agent <-> backend tunnel. Keep in sync with
// backend/Services/RemoteShell/ShellProtocol.cs.
//
// Control messages: JSON text frames {"type": "..."}.
//   agent -> backend: hello{version,commit}, ping, exited{sessionId,code}
//   backend -> agent: open{sessionId,cols,rows}, resize{sessionId,cols,rows}, close{sessionId}, pong
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

}  // namespace bloom_agent
