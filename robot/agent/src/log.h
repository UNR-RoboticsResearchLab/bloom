// Minimal logging to stderr. The "<N>" prefixes are syslog priorities, which journald parses
// (SyslogLevelPrefix=yes is the default), so `journalctl -u bloom-agent -p warning` works.
#pragma once

#include <string>

namespace bloom_agent::log {
void info(const std::string& msg);
void warn(const std::string& msg);
void error(const std::string& msg);
}  // namespace bloom_agent::log
