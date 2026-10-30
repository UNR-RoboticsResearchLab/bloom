// SemVer 2.0 parsing and precedence (semver.org section 11). Versions are compared as SemVer,
// never as strings: "1.0.10" > "1.0.9", and "1.0.5-g1a2b3c" < "1.0.5". Used for the version the
// agent reports now and by OTA (Phase 2), which compares against the backend's release registry.
#pragma once

#include <cctype>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bloom_agent {

struct SemVer {
  std::uint64_t major = 0;
  std::uint64_t minor = 0;
  std::uint64_t patch = 0;
  std::vector<std::string> prerelease;  // dot-separated identifiers
  std::string build;                    // ignored for precedence

  static std::optional<SemVer> parse(std::string_view text) {
    SemVer v;
    const auto plus = text.find('+');
    if (plus != std::string_view::npos) {
      v.build = std::string(text.substr(plus + 1));
      if (v.build.empty() || !valid_identifiers(v.build, /*numeric_rules=*/false)) return std::nullopt;
      text = text.substr(0, plus);
    }
    const auto dash = text.find('-');
    if (dash != std::string_view::npos) {
      const auto pre = text.substr(dash + 1);
      if (pre.empty() || !valid_identifiers(pre, /*numeric_rules=*/true)) return std::nullopt;
      v.prerelease = split(pre);
      text = text.substr(0, dash);
    }
    const auto parts = split(text);
    if (parts.size() != 3) return std::nullopt;
    std::uint64_t* fields[] = {&v.major, &v.minor, &v.patch};
    for (int i = 0; i < 3; ++i) {
      if (!is_numeric(parts[i]) || (parts[i].size() > 1 && parts[i][0] == '0')) return std::nullopt;
      *fields[i] = std::stoull(parts[i]);
    }
    return v;
  }

  // <0, 0, >0 like strcmp.
  int compare(const SemVer& other) const {
    if (major != other.major) return major < other.major ? -1 : 1;
    if (minor != other.minor) return minor < other.minor ? -1 : 1;
    if (patch != other.patch) return patch < other.patch ? -1 : 1;
    // A version without prerelease has higher precedence than one with.
    if (prerelease.empty() || other.prerelease.empty()) {
      if (prerelease.empty() == other.prerelease.empty()) return 0;
      return prerelease.empty() ? 1 : -1;
    }
    for (size_t i = 0; i < prerelease.size() && i < other.prerelease.size(); ++i) {
      const auto& a = prerelease[i];
      const auto& b = other.prerelease[i];
      const bool an = is_numeric(a), bn = is_numeric(b);
      if (an && bn) {
        const auto x = std::stoull(a), y = std::stoull(b);
        if (x != y) return x < y ? -1 : 1;
      } else if (an != bn) {
        return an ? -1 : 1;  // numeric identifiers sort before alphanumeric
      } else if (a != b) {
        return a < b ? -1 : 1;
      }
    }
    if (prerelease.size() == other.prerelease.size()) return 0;
    return prerelease.size() < other.prerelease.size() ? -1 : 1;
  }

  bool operator<(const SemVer& o) const { return compare(o) < 0; }
  bool operator>(const SemVer& o) const { return compare(o) > 0; }
  bool operator==(const SemVer& o) const { return compare(o) == 0; }
  bool operator!=(const SemVer& o) const { return compare(o) != 0; }

 private:
  static bool is_numeric(std::string_view s) {
    if (s.empty()) return false;
    for (char c : s)
      if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    return true;
  }

  static std::vector<std::string> split(std::string_view s) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
      const auto dot = s.find('.', start);
      out.emplace_back(s.substr(start, dot - start));
      if (dot == std::string_view::npos) return out;
      start = dot + 1;
    }
  }

  static bool valid_identifiers(std::string_view s, bool numeric_rules) {
    for (const auto& id : split(s)) {
      if (id.empty()) return false;
      for (char c : id)
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-') return false;
      if (numeric_rules && is_numeric(id) && id.size() > 1 && id[0] == '0') return false;
    }
    return true;
  }
};

}  // namespace bloom_agent
