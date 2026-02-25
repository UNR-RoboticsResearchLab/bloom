#include "bloom_node/behavior_coordinator.h"

using namespace bloom_node;

BehaviorCoordinator::BehaviorCoordinator() = default;

void BehaviorCoordinator::request_behavior(const std::string &name, int priority, bool interrupt) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if this behavior conflicts with current behavior
  if (!current_behavior_.empty() && !interrupt) {
    // Non-interruptible request when something is running - queue it
    pending_behaviors_.push({name, priority, std::chrono::system_clock::now(), interrupt});
    return;
  }

  // Check for exclusive group conflicts
  for (const auto &[group, behaviors] : exclusive_groups_) {
    auto it = std::find(behaviors.begin(), behaviors.end(), name);
    if (it != behaviors.end()) {
      // This behavior is in an exclusive group
      if (!current_behavior_.empty()) {
        auto current_it = std::find(behaviors.begin(), behaviors.end(), current_behavior_);
        if (current_it != behaviors.end() && !interrupt) {
          // Current behavior is in same exclusive group and not interruptible
          pending_behaviors_.push({name, priority, std::chrono::system_clock::now(), interrupt});
          return;
        }
      }
      break;
    }
  }

  // Can execute immediately
  current_behavior_ = name;
}

std::string BehaviorCoordinator::get_next_behavior() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (pending_behaviors_.empty()) {
    return "";
  }

  auto next = pending_behaviors_.top();
  pending_behaviors_.pop();

  // Check if we can interrupt current behavior
  if (!current_behavior_.empty() && !next.interrupt_current) {
    // Can't interrupt - put it back and return empty
    pending_behaviors_.push(next);
    return "";
  }

  current_behavior_ = next.name;
  return next.name;
}

void BehaviorCoordinator::behavior_completed(const std::string &name) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_behavior_ == name) {
    current_behavior_.clear();
  }
}

bool BehaviorCoordinator::is_behavior_running(const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_behavior_ == name;
}

void BehaviorCoordinator::set_exclusive_group(const std::string &group_name,
                                             const std::vector<std::string> &behaviors) {
  std::lock_guard<std::mutex> lock(mutex_);
  exclusive_groups_[group_name] = behaviors;
}

size_t BehaviorCoordinator::pending_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_behaviors_.size();
}

void BehaviorCoordinator::clear_pending() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Clear priority queue by creating a new empty one
  std::priority_queue<BehaviorRequest> empty;
  std::swap(pending_behaviors_, empty);
}
