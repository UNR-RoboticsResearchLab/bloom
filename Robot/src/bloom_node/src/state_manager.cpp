#include "bloom_node/state_manager.h"

using namespace bloom_node;
using namespace std::chrono_literals;

StateManager::StateManager(const rclcpp::NodeOptions & options)
	: rclcpp::Node("state_manager", options)
{
	// Publishers
	state_pub_ = this->create_publisher<std_msgs::msg::String>("robot/state", 10);
	behavior_pub_ = this->create_publisher<std_msgs::msg::String>("play_sequence", 10);
	face_pub_ = this->create_publisher<std_msgs::msg::String>("robot/face_state", 10);

	// Subscription to accept state change commands
	state_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
		"robot/state_cmd", 10,
		[this](std_msgs::msg::String::SharedPtr msg) { this->on_state_cmd(msg); });

	// Subscription to sequence_status to drive behavior looping
	sequence_status_sub_ = this->create_subscription<std_msgs::msg::String>(
		"sequence_status", 10,
		[this](std_msgs::msg::String::SharedPtr msg) { this->on_sequence_status(msg); });

	// Service to trigger behaviors for the current state
	trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
		"robot/trigger_behaviors",
		[this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
			bool ok = this->trigger_behaviors();
			res->success = ok;
			res->message = ok ? "Behaviors triggered" : "No behaviors for current state";
		});


	// Heartbeat timer: periodically publish current state
	heartbeat_timer_ = this->create_wall_timer(1s, [this]() { this->publish_state(); });

	// Default in-memory state -> behavior mapping. Replaceable via set_state_behaviors().
	set_state_behaviors({
		{"waiting", {"idle"}},
		{"talking", {"visyme_sync"}},
        {"loading", {"bringup"}},
		{"listening", {"listening_bobble", "look_up", "look_left"}},
		{"happy", {"happy"}},
		{"affirm", {"affirm"}},
		{"reject", {"reject"}},
		{"correct", {"look_up"}},
		{"incorrect", {"look_down"}}
	});

	// Default state -> face expression mapping
	set_state_face_expressions({
		{"waiting", "neutral"},
		{"talking", "smile"},
		{"loading", "neutral"},
		{"listening", "engaged"},
		{"happy", "smile"},
		{"affirm", "warm_smile"},
		{"reject", "thoughtful"},
		{"correct", "playful"},
		{"incorrect", "thoughtful"}
	});

	// Initialize state
	{
		std::lock_guard<std::mutex> lk(mutex_);
		current_state_ = "idle";
	}
	publish_state();

	RCLCPP_INFO(this->get_logger(), "StateManager node started (initial state: %s)", current_state_.c_str());
}

void StateManager::set_state(const std::string & state, int timing_ms)
{
	if (state.empty()) return;
	std::string previous;
	{
		std::lock_guard<std::mutex> lk(mutex_);
		previous = current_state_;
		// if (state == current_state_) {
		// 	RCLCPP_DEBUG(this->get_logger(), "set_state called but state unchanged: %s", state.c_str());
		// 	return;
		// }
		current_state_ = state;
	}

	RCLCPP_INFO(this->get_logger(), "State changed: %s -> %s", previous.c_str(), state.c_str());
	publish_state();

	// Publish face expression for the new state
	auto it = state_faces_.find(state);
	if (it != state_faces_.end()) {
		publish_face(it->second);
	}

	// Stop any running behavior loop and start a new one for this state
	stop_behavior_loop();
	auto behaviors = get_behaviors_for_state(state);
	start_behavior_loop(behaviors);

	// If timing_ms is specified, set a timer to stop behavior loop after that duration
	if (timing_ms > 0) {
		// Cancel any existing behavior loop timer
		if (behavior_loop_timer_) {
			behavior_loop_timer_->cancel();
		}

		// Create a new timer to stop the behavior loop
		behavior_loop_timer_ = this->create_wall_timer(
			std::chrono::milliseconds(timing_ms),
			[this]() {
				RCLCPP_DEBUG(this->get_logger(), "Behavior loop timer expired, stopping loop");
				stop_behavior_loop();
			});

		RCLCPP_INFO(this->get_logger(), "Set behavior loop to stop after %d ms", timing_ms);
	}
}

std::string StateManager::get_state() const
{
	std::lock_guard<std::mutex> lk(mutex_);
	return current_state_;
}

std::vector<std::string> StateManager::get_behaviors_for_state(const std::string & state) const
{
	std::lock_guard<std::mutex> lk(mutex_);
	auto it = state_behaviors_.find(state);
	if (it == state_behaviors_.end()) return {};
	return it->second;
}

bool StateManager::trigger_behaviors()
{
	auto behaviors = get_behaviors_for_state(get_state());
	if (behaviors.empty()) {
		RCLCPP_WARN(this->get_logger(), "No behaviors for state: %s", get_state().c_str());
		return false;
	}

	for (const auto &b : behaviors) {
		publish_behavior(b);
	}
	return true;
}

void StateManager::set_state_behaviors(const std::unordered_map<std::string, std::vector<std::string>> & mapping)
{
	std::lock_guard<std::mutex> lk(mutex_);
	state_behaviors_ = mapping;
}

void StateManager::on_state_cmd(const std_msgs::msg::String::SharedPtr msg)
{
	if (!msg) return;
	set_state(msg->data);
}

void StateManager::publish_state()
{
	std_msgs::msg::String msg;
	msg.data = get_state();
	if (state_pub_) state_pub_->publish(msg);
}

void StateManager::publish_behavior(const std::string & behavior_name)
{
	std_msgs::msg::String msg;
	msg.data = behavior_name;
	if (behavior_pub_) behavior_pub_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "Published behavior execute request: %s", behavior_name.c_str());
}

void StateManager::publish_face(const std::string & face_expression)
{
	std_msgs::msg::String msg;
	msg.data = face_expression;
	if (face_pub_) face_pub_->publish(msg);
	RCLCPP_INFO(this->get_logger(), "Published face expression: %s", face_expression.c_str());
}

void StateManager::set_state_face_expressions(const std::unordered_map<std::string, std::string> & mapping)
{
	std::lock_guard<std::mutex> lk(mutex_);
	state_faces_ = mapping;
}

void StateManager::start_behavior_loop(const std::vector<std::string> & behaviors)
{
	if (behaviors.empty()) {
		RCLCPP_WARN(this->get_logger(), "start_behavior_loop called with empty behaviors list");
		return;
	}

	std::string first_behavior;
	std::string state_name;
	{
		std::lock_guard<std::mutex> lk(mutex_);

		// Copy and shuffle behaviors
		shuffled_behaviors_ = behaviors;
		std::shuffle(shuffled_behaviors_.begin(), shuffled_behaviors_.end(), rng_);
		behavior_index_ = 1;  // Start at 1 since we'll publish the first one now

		if (!shuffled_behaviors_.empty()) {
			first_behavior = shuffled_behaviors_[0];
			state_name = current_state_;
			RCLCPP_INFO(this->get_logger(), "Starting behavior loop for state: %s with %zu behaviors",
				state_name.c_str(), shuffled_behaviors_.size());
		}
	}

	// Publish first behavior outside the lock to avoid deadlock
	if (!first_behavior.empty()) {
		publish_behavior(first_behavior);
	}
}

void StateManager::stop_behavior_loop()
{
	std::lock_guard<std::mutex> lk(mutex_);
	shuffled_behaviors_.clear();
	behavior_index_ = 0;
}

void StateManager::on_behavior_loop_tick()
{
	std::string next_behavior;
	{
		std::lock_guard<std::mutex> lk(mutex_);

		if (shuffled_behaviors_.empty()) {
			return;
		}

		// If we've gone through all behaviors, re-shuffle
		if (behavior_index_ >= shuffled_behaviors_.size()) {
			std::shuffle(shuffled_behaviors_.begin(), shuffled_behaviors_.end(), rng_);
			behavior_index_ = 0;
		}

		next_behavior = shuffled_behaviors_[behavior_index_];
		behavior_index_++;
	}

	// Publish outside the lock to avoid deadlock
	publish_behavior(next_behavior);
}

void StateManager::on_sequence_status(const std_msgs::msg::String::SharedPtr msg)
{
	if (!msg) return;

	// Parse the message: format is "completed:sequence_name" or "playing:sequence_name"
	const std::string & data = msg->data;
	if (data.find("completed:") == 0) {
		// A sequence has completed, publish the next behavior in the loop
		on_behavior_loop_tick();
	}
}
