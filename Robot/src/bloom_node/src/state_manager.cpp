#include "bloom_node/state_manager.h"

namespace state_manager {

using namespace std::chrono_literals;

StateManager::StateManager(const rclcpp::NodeOptions & options)
	: rclcpp::Node("state_manager", options)
{
	// Publishers
	state_pub_ = this->create_publisher<std_msgs::msg::String>("robot/state", 10);
	behavior_pub_ = this->create_publisher<std_msgs::msg::String>("robot/behavior/execute", 10);

	// Subscription to accept state change commands
	state_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
		"robot/state_cmd", 10,
		[this](std_msgs::msg::String::SharedPtr msg) { this->on_state_cmd(msg); });

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
		{"waiting", {"idle_animation"}},
		{"talking", {"visyme_sync"}},
        {"loading", {"load_animation"}}
	});

	// Initialize state
	{
		std::lock_guard<std::mutex> lk(mutex_);
		current_state_ = "idle";
	}
	publish_state();

	RCLCPP_INFO(this->get_logger(), "StateManager node started (initial state: %s)", current_state_.c_str());
}

void StateManager::set_state(const std::string & state)
{
	if (state.empty()) return;
	std::string previous;
	{
		std::lock_guard<std::mutex> lk(mutex_);
		previous = current_state_;
		if (state == current_state_) {
			RCLCPP_DEBUG(this->get_logger(), "set_state called but state unchanged: %s", state.c_str());
			return;
		}
		current_state_ = state;
	}

	RCLCPP_INFO(this->get_logger(), "State changed: %s -> %s", previous.c_str(), state.c_str());
	publish_state();
    trigger_behaviors();
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

} // namespace state_manager


#ifdef STATE_MANAGER_MAIN
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<state_manager::StateManager>(rclcpp::NodeOptions());
	RCLCPP_INFO(node->get_logger(), "Starting standalone StateManager...");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
#endif
