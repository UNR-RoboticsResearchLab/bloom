// Standard base64 (RFC 4648 section 4, with padding). SignalR's JSON protocol carries byte[] as a
// base64 string, so terminal bytes travel in it; it's also used for HTTP proxy credentials.
#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace bloom_agent {

std::string base64_encode(std::string_view input);

// nullopt for anything that isn't well-formed base64 (bad characters, bad length or padding).
std::optional<std::string> base64_decode(std::string_view input);

}  // namespace bloom_agent
