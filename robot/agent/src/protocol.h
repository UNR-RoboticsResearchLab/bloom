// The agent's side of the remote shell's hub contract, carried by the SignalR JSON protocol
// (signalr.h). Keep in sync with backend/Services/RemoteShell/{RobotAgentHub,ShellHubClients}.cs.
//
//   agent -> server (hub methods):  Hello(version, commit)  Output(sessionId, bytes)
//                                   SessionExited(sessionId, code)  SecretApplied(requestId, ok, message)
//   server -> agent (client methods): Open(sessionId, cols, rows)  Input(sessionId, bytes)
//                                   Resize(sessionId, cols, rows)  Close(sessionId)
//                                   SetSecret(requestId, secret)  Disconnecting(reason)
//
// GUIDs are strings and byte[] is base64 in the JSON protocol. SetSecret carries a secret: never log it.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace bloom_agent {

using Uuid = std::array<std::uint8_t, 16>;

// Accepts the canonical 8-4-4-4-12 form (what .NET's Guid serializes to).
std::optional<Uuid> parse_uuid(std::string_view text);
std::string format_uuid(const Uuid& id);

// ── agent -> server: each returns a complete, framed message ready to send ───

std::string hello_message(std::string_view version, std::string_view commit);
std::string output_message(const Uuid& session, std::string_view bytes);
std::string exited_message(const Uuid& session, std::optional<int> code);
std::string secret_applied_message(std::string_view request_id, bool ok, std::string_view message);

// ── server -> agent: read positional arguments of an invocation ──────────────

std::optional<Uuid> arg_uuid(const nlohmann::json& args, std::size_t index);
std::optional<int> arg_int(const nlohmann::json& args, std::size_t index);
std::optional<std::string> arg_string(const nlohmann::json& args, std::size_t index);
std::optional<std::string> arg_bytes(const nlohmann::json& args, std::size_t index);  // base64-decoded

struct SetSecretRequest {
  std::string request_id;
  std::string secret;
};
// nullopt unless there's a GUID requestId and a secret made only of the characters the backend
// generates (base64url, 16..512 long), so nothing odd can reach a shell or a file.
std::optional<SetSecretRequest> parse_set_secret(const nlohmann::json& args);
// The requestId of a SetSecret even when its secret is invalid, so the backend still gets an answer
// instead of waiting out its timeout.
std::optional<std::string> request_id_of(const nlohmann::json& args);

}  // namespace bloom_agent
