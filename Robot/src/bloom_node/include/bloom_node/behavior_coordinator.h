#ifndef BLOOM_NODE_BEHAVIOR_COORDINATOR_H
#define BLOOM_NODE_BEHAVIOR_COORDINATOR_H

#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <mutex>
#include <chrono>
#include <algorithm>

namespace bloom_node {

/**
 * Behavior priority and execution tracking for semi-autonomous control.
 */
struct BehaviorRequest {
  std::string name;
  int priority;  // Higher = more important
  std::chrono::system_clock::time_point timestamp;
  bool interrupt_current;  // interrupt running behavior?

  bool operator<(const BehaviorRequest &other) const {
    // Priority queue orders by priority (higher first), then by timestamp
    if (priority != other.priority) {
      return priority < other.priority;  // Reverse: higher priority first
    }
    return timestamp > other.timestamp;  // Earlier timestamps first
  }
};

/**
 * Coordinates behavior execution with priority arbitration.
 * Prevents conflicting behaviors from running simultaneously and enforces priorities.
 */
class BehaviorCoordinator {
public:
  explicit BehaviorCoordinator();

  /// Request a behavior execution with optional priority and interrupt flag
  void request_behavior(const std::string &name, int priority = 0, bool interrupt = false);

  /// Get the next highest priority behavior to execute (or empty string if none)
  std::string get_next_behavior();

  /// Queue a behavior directly (used by LessonCoordinator)
  void queue_behavior(const std::string &name);

  /// Mark a behavior as completed
  void behavior_completed(const std::string &name);

  /// Check if a behavior is currently running
  bool is_behavior_running(const std::string &name) const;

  /// Set behavior exclusivity groups (behaviors in same group are mutually exclusive)
  void set_exclusive_group(const std::string &group_name,
                          const std::vector<std::string> &behaviors);

  /// Get pending behavior count (for monitoring)
  size_t pending_count() const;

  /// Clear all pending behaviors
  void clear_pending();


private:
  mutable std::mutex mutex_;
  std::priority_queue<BehaviorRequest> pending_behaviors_;
  std::string current_behavior_;
  std::unordered_map<std::string, std::vector<std::string>> exclusive_groups_;

  bool can_interrupt(const std::string &new_behavior) const;
};

} // namespace bloom_node

#endif // BLOOM_NODE_BEHAVIOR_COORDINATOR_H
