#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "bloom_msgs/action/play_behavior.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <chrono>
#include <sstream>
#include <random>
#include <algorithm>

namespace bloom_node {

/// StateManager is a small ROS2 node encapsulating a robot state machine and
/// a mapping from states to behaviors. It provides:
/// - a publisher for the current state (`robot/state`)
/// - a subscription to accept state change commands (`robot/state_cmd`)
/// - a publisher for behavior execution requests (`robot/behavior/execute`)
/// - a service to trigger behaviors for the current state (`robot/trigger_behaviors`)
/// - a publisher for face state (TODO), images, and other relevant lesson info (TODO)
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

	// Set the robot state. This will update the internal state and publish it.
	// example states: "waiting", "talking", "loading", "listening", etc.
	// Optional: provide timing_ms to auto-stop the behavior loop after that duration
	void set_state(const std::string & state, int timing_ms = 0);

	// Get the current robot state (thread-safe read).
	std::string get_state() const;

	// Return the list of behavior names mapped to `state`. If none found,
	// returns an empty vector.
	std::vector<std::string> get_behaviors_for_state(const std::string & state) const;

	// Trigger all behaviors mapped to the current state. This will publish
	// behavior execution messages and return a boolean success indicator.
	bool trigger_behaviors();

	// Load or replace state->behavior mappings from a parameter or a provided
	// map. Implementations may choose to read from params on construction.
	void set_state_behaviors(const std::unordered_map<std::string, std::vector<std::string>> & mapping);

	// Load or replace state->face_expression mappings.
	void set_state_face_expressions(const std::unordered_map<std::string, std::string> & mapping);

private:
	// Callback invoked when a String message is received on the state_cmd topic.
	void on_state_cmd(const std_msgs::msg::String::SharedPtr msg);

	// Callback invoked when a sequence_status message is received.
	void on_sequence_status(const std_msgs::msg::String::SharedPtr msg);

	// Publish the current state on the state topic.
	void publish_state();

	// Publish a single behavior execution request for `behavior_name`.
	void publish_behavior(const std::string & behavior_name);

	// Publish a face expression.
	void publish_face(const std::string & face_expression);

	// Start the behavior loop: shuffle behaviors and cycle through them.
	void start_behavior_loop(const std::vector<std::string> & behaviors);

	// Stop the behavior loop timer.
	void stop_behavior_loop();

	// Called periodically to publish the next behavior in the shuffled sequence.
	void on_behavior_loop_tick();

	// ROS interfaces
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
	rclcpp_action::Client<bloom_msgs::action::PlayBehavior>::SharedPtr behavior_action_client_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_pub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_cmd_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sequence_status_sub_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
	rclcpp::TimerBase::SharedPtr heartbeat_timer_;
	rclcpp::TimerBase::SharedPtr behavior_loop_timer_;  // Timer to auto-stop behavior loop after timing_seconds

	// State and mapping (protected by mutex for thread-safety)
	mutable std::mutex mutex_;
	std::string current_state_;
	std::unordered_map<std::string, std::vector<std::string>> state_behaviors_;
	std::unordered_map<std::string, std::string> state_faces_;

	// Behavior looping state
	std::vector<std::string> shuffled_behaviors_;
	std::size_t behavior_index_{0};

	// Random number generator
	std::mt19937 rng_{std::random_device{}()};
};

} // namespace bloom_node

#endif