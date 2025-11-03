#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <chrono>
#include <sstream>

namespace state_manager {

/// StateManager is a small ROS2 node encapsulating a robot state machine and
/// a mapping from states to behaviors. It provides:
/// - a publisher for the current state (`robot/state`)
/// - a subscription to accept state change commands (`robot/state_cmd`)
/// - a publisher for behavior execution requests (`robot/behavior/execute`)
/// - a service to trigger behaviors for the current state (`robot/trigger_behaviors`)
///
/// This header provides the class declaration. The implementation can be
/// placed in a corresponding .cpp file or used inline for small projects.
class StateManager : public rclcpp::Node
{
public:
	using Ptr = std::shared_ptr<StateManager>;

	/// Construct the node. Optionally provide NodeOptions for composition.
	explicit StateManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

	~StateManager() override = default;

	// Non-copyable
	StateManager(const StateManager &) = delete;
	StateManager & operator=(const StateManager &) = delete;

	/// Set the robot state. This will update the internal state and publish it.
	void set_state(const std::string & state);

	/// Get the current robot state (thread-safe read).
	std::string get_state() const;

	/// Return the list of behavior names mapped to `state`. If none found,
	/// returns an empty vector.
	std::vector<std::string> get_behaviors_for_state(const std::string & state) const;

	/// Trigger all behaviors mapped to the current state. This will publish
	/// behavior execution messages and return a boolean success indicator.
	bool trigger_behaviors();

	/// Load or replace state->behavior mappings from a parameter or a provided
	/// map. Implementations may choose to read from params on construction.
	void set_state_behaviors(const std::unordered_map<std::string, std::vector<std::string>> & mapping);

private:
	/// Callback invoked when a String message is received on the state_cmd topic.
	void on_state_cmd(const std_msgs::msg::String::SharedPtr msg);

	/// Publish the current state on the state topic.
	void publish_state();

	/// Publish a single behavior execution request for `behavior_name`.
	void publish_behavior(const std::string & behavior_name);

	// ROS interfaces
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_cmd_sub_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
	rclcpp::TimerBase::SharedPtr heartbeat_timer_;

	// State and mapping (protected by mutex for thread-safety)
	mutable std::mutex mutex_;
	std::string current_state_;
	std::unordered_map<std::string, std::vector<std::string>> state_behaviors_;
};

} // namespace state_manager

#endif