#ifndef BLOOM_NODE_LESSON_POLLER_H
#define BLOOM_NODE_LESSON_POLLER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <atomic>
#include <string>
#include <functional>
#include "bloom_node/web_service_client.h"
#include "bloom_node/lessson_coordinator.h"
#include "bloom_node/json.hpp"

namespace bloom_node {

using json = nlohmann::json;

/**
 * LessonPoller: Polling service for backend-driven lesson execution.
 *
 * Periodically polls the backend for pending lessons and auto-starts them
 * via LessonCoordinator. Implements deduplication to prevent duplicate lesson
 * execution and waits for lesson completion before accepting new lessons.
 *
 * Polling endpoint: GET /api/robotsessions/{sessionId}/pending-lesson
 * Response format:
 *   {
 *     "hasPendingLesson": true,
 *     "lesson": { ... full lesson JSON ... }
 *   }
 * Or 204 No Content if no pending lesson.
 */
class LessonPoller : public rclcpp::Node {

public:

	LessonPoller(
		std::shared_ptr<WebServiceClient> web_client,
		std::shared_ptr<LessonCoordinator> lesson_coord,
		const std::string &session_id,
		int poll_interval_ms = 7000,
		const std::string &node_name = "lesson_poller"
	);

	~LessonPoller() override;

	// Non-copyable
	LessonPoller(const LessonPoller &) = delete;
	LessonPoller & operator=(const LessonPoller &) = delete;

	/// Start the polling loop
	void start_polling();

	/// Stop the polling loop
	void stop_polling();

	/// Dynamically update the session ID
	bool set_session_id(const std::string &session_id);

	void set_pairing_code(const std::string &pairing_code);

private:
	/// Timer callback - executed every poll_interval_ms
	void on_polling_tick();

	/// Handle retrieved pending lesson data
	void handle_pending_lesson(const json &lesson_json);

	// Dependencies
	std::shared_ptr<WebServiceClient> web_client_;
	std::shared_ptr<LessonCoordinator> lesson_coord_;

	// State
	std::string session_id_;
	std::atomic<bool> currently_executing_{false};
	std::string last_lesson_id_;  // For deduplication
	int poll_interval_ms_;

	// ROS2 timer for polling
	rclcpp::TimerBase::SharedPtr poll_timer_;

	// ROS2 subscription for direct lesson loading (backendless)
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_lesson_sub_;

	// Mutex for thread-safe session_id updates
	mutable std::mutex session_mutex_;

	std::atomic<bool> paired_{false};
	std::string pairing_code_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr session_code_pub_;
};

}  // namespace bloom_node

#endif  // BLOOM_NODE_LESSON_POLLER_H
