#pragma once

#include <string>

namespace bloom_agent {
// Prints a check-by-check report; returns the process exit code (0 = all passed).
int run_doctor(const std::string& config_path);
}  // namespace bloom_agent
