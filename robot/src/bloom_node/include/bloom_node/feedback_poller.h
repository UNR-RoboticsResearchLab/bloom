#ifndef BLOOM_NODE_FEEDBACK_POLLER_H
#define BLOOM_NODE_FEEDBACK_POLLER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <atomic>
#include <string>
#include <functional>
#include "bloom_node/web_service_client.h"
#include "bloom_node/json.hpp"

namespace bloom_node {

using json = nlohmann::json;

/**
 * FeedbackPoller: Polling service for SLP feedback (approve/retry) commands.
 *
 * Periodically polls the backend for pending feedback issued by the Speech-Language
 * Pathologist (SLP) during lesson execution. When feedback is found, it publishes
 * it to ROS topics and acknowledges receipt to prevent re-execution.
 *
 * Polling endpoint: GET /api/robotsessions/{sessionId}/pending-feedback
 * Acknowledgment endpoint: PUT /api/robotsessions/{sessionId}/pending-feedback/{feedbackId}/acknowledge
 *
 * Response format:
 *   {
 *     "feedbackId": "guid",
 *     "sessionId": "guid",
 *     "stepId": 1,
 *     "command": "approve" | "retry",
 *     "timestamp": "2025-03-04T...",
 *     "acknowledged": false
 *   }
 * Or 204 No Content if no pending feedback.
 */
class FeedbackPoller : public rclcpp::Node {

public:

	FeedbackPoller(
		std::shared_ptr<WebServiceClient> web_client,
		const std::string &session_id,
		int poll_interval_ms = 1000,
		const std::string &node_name = "feedback_poller"
	);

	~FeedbackPoller() override;

	// Non-copyable
	FeedbackPoller(const FeedbackPoller &) = delete;
	FeedbackPoller & operator=(const FeedbackPoller &) = delete;

	/// Start the polling loop
	void start_polling();

	/// Stop the polling loop
	void stop_polling();

	/// Dynamically update the session ID
	bool set_session_id(const std::string &session_id);

	/// Register callback to receive feedback commands
	void set_feedback_callback(std::function<void(const json &feedback)> callback);

	/// Enable/disable polling (paused when not waiting for interaction)
	void set_polling_active(bool active);

	/// Check if polling is currently active
	bool is_polling_active() const;

private:
	/// Timer callback - executed every poll_interval_ms
	void on_polling_tick();

	/// Handle retrieved pending feedback data
	void handle_pending_feedback(const json &feedback_json);

	/// Acknowledge feedback to prevent re-execution
	void acknowledge_feedback(const std::string &session_id, const std::string &feedback_id);

	// Dependencies
	std::shared_ptr<WebServiceClient> web_client_;

	// State
	std::string session_id_;
	std::string last_feedback_id_;  // For deduplication
	int poll_interval_ms_;
	std::atomic<bool> polling_active_{false};  // Only poll when lesson is waiting for interaction

	// ROS2 timer for polling
	rclcpp::TimerBase::SharedPtr poll_timer_;

	// ROS2 publishers for feedback commands
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;

	// User callback for feedback events
	std::function<void(const json &feedback)> feedback_callback_;

	// Mutex for thread-safe session_id updates
	mutable std::mutex session_mutex_;

	// Mutex for feedback callback
	mutable std::mutex callback_mutex_;
};

}  // namespace bloom_node

#endif  // BLOOM_NODE_FEEDBACK_POLLER_H
