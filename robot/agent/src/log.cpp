#include "log.h"

#include <cstdio>

namespace bloom_agent::log {
namespace {
void emit(int priority, const std::string& msg) {
  std::fprintf(stderr, "<%d>%s\n", priority, msg.c_str());
  std::fflush(stderr);
}
}  // namespace

void info(const std::string& msg) { emit(6, msg); }
void warn(const std::string& msg) { emit(4, msg); }
void error(const std::string& msg) { emit(3, msg); }
}  // namespace bloom_agent::log
