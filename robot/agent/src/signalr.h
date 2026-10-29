// The SignalR JSON hub protocol, as much of it as this agent needs (the client side of a hub
// connection over an already-open WebSocket). There is no C++ SignalR client from Microsoft, and the
// agent must not depend on .NET, so it speaks the protocol itself:
//   https://github.com/dotnet/aspnetcore/blob/main/src/SignalR/docs/specs/HubProtocol.md
//
// Every message is a JSON object followed by the record separator 0x1E, and a WebSocket message may
// hold several of them. The agent connects straight to the WebSocket without SignalR's negotiate
// step (the "skipNegotiation" mode, valid when WebSockets is the only transport).
#pragma once

#include <optional>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace bloom_agent::signalr {

constexpr char kRecordSeparator = '\x1e';

// First thing sent after the WebSocket opens: {"protocol":"json","version":1}<RS>
std::string handshake_request();

struct HandshakeResponse {
  bool ok = false;
  std::string error;  // set when the server refused (e.g. unknown protocol)
};
// `record` is one record without its separator; the server answers `{}` on success.
HandshakeResponse parse_handshake_response(std::string_view record);

// Frames a fire-and-forget invocation (no invocationId, so the server sends no completion).
std::string invocation(std::string_view target, const nlohmann::json& arguments);
// {"type":6}<RS>, which both ends send periodically. Any traffic counts as liveness.
std::string ping();

// Splits a byte stream into records. Holds a partial record until its separator arrives.
class RecordParser {
 public:
  void feed(std::string_view data) { buffer_.append(data); }
  std::optional<std::string> next();

 private:
  std::string buffer_;
};

enum class MessageType { Unknown = 0, Invocation = 1, StreamItem = 2, Completion = 3, StreamInvocation = 4,
                         CancelInvocation = 5, Ping = 6, Close = 7 };

struct Message {
  MessageType type = MessageType::Unknown;
  std::string target;        // Invocation
  nlohmann::json arguments;  // Invocation: always an array
  std::string error;         // Close, Completion
};
// nullopt for anything that isn't a JSON object with a numeric "type".
std::optional<Message> parse_message(std::string_view record);

}  // namespace bloom_agent::signalr
