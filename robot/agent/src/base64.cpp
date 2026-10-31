#include "base64.h"

#include <array>
#include <cstdint>

namespace bloom_agent {
namespace {
constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

constexpr std::array<std::int8_t, 256> make_reverse() {
  std::array<std::int8_t, 256> r{};
  for (auto& v : r) v = -1;
  for (int i = 0; i < 64; ++i) r[static_cast<unsigned char>(kTable[i])] = static_cast<std::int8_t>(i);
  return r;
}
constexpr auto kReverse = make_reverse();
}  // namespace

std::string base64_encode(std::string_view input) {
  std::string out;
  out.reserve((input.size() + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < input.size(); i += 3) {
    const unsigned n = static_cast<unsigned char>(input[i]) << 16 | static_cast<unsigned char>(input[i + 1]) << 8 |
                       static_cast<unsigned char>(input[i + 2]);
    out += {kTable[n >> 18], kTable[(n >> 12) & 63], kTable[(n >> 6) & 63], kTable[n & 63]};
  }
  if (i + 1 == input.size()) {
    const unsigned n = static_cast<unsigned char>(input[i]) << 16;
    out += {kTable[n >> 18], kTable[(n >> 12) & 63], '=', '='};
  } else if (i + 2 == input.size()) {
    const unsigned n = static_cast<unsigned char>(input[i]) << 16 | static_cast<unsigned char>(input[i + 1]) << 8;
    out += {kTable[n >> 18], kTable[(n >> 12) & 63], kTable[(n >> 6) & 63], '='};
  }
  return out;
}

std::optional<std::string> base64_decode(std::string_view input) {
  if (input.size() % 4 != 0) return std::nullopt;
  std::string out;
  out.reserve(input.size() / 4 * 3);
  for (size_t i = 0; i < input.size(); i += 4) {
    const bool last = i + 4 == input.size();
    int v[4];
    int pad = 0;
    for (int j = 0; j < 4; ++j) {
      const char c = input[i + j];
      if (c == '=') {
        // Padding only in the last quantum, only at its end.
        if (!last || j < 2) return std::nullopt;
        ++pad;
        v[j] = 0;
      } else {
        if (pad > 0) return std::nullopt;
        const int d = kReverse[static_cast<unsigned char>(c)];
        if (d < 0) return std::nullopt;
        v[j] = d;
      }
    }
    const unsigned n = static_cast<unsigned>(v[0]) << 18 | static_cast<unsigned>(v[1]) << 12 |
                       static_cast<unsigned>(v[2]) << 6 | static_cast<unsigned>(v[3]);
    out.push_back(static_cast<char>(n >> 16));
    if (pad < 2) out.push_back(static_cast<char>((n >> 8) & 0xff));
    if (pad < 1) out.push_back(static_cast<char>(n & 0xff));
  }
  return out;
}

}  // namespace bloom_agent
