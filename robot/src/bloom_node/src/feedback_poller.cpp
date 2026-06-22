#include "bloom_node/feedback_poller.h"
#include <rclcpp/logging.hpp>
#include <sstream>

using namespace bloom_node;
using namespace std::chrono_literals;

FeedbackPoller::FeedbackPoller(
	std::shared_ptr<WebServiceClient> web_client,
	const std::string &session_id,
	int poll_interval_ms,
	const std::string &node_name
)
	: rclcpp::Node(node_name),
	  web_client_(web_client),
	  session_id_(session_id),
	  poll_interval_ms_(poll_interval_ms) {
	RCLCPP_INFO(this->get_logger(), "FeedbackPoller initialized (session_id: %s, poll_interval: %dms)",
		session_id_.c_str(), poll_interval_ms_);

	// Create publisher for feedback commands
	feedback_pub_ = this->create_publisher<std_msgs::msg::String>(
		"/robot/pending_feedback", 10);

	RCLCPP_INFO(this->get_logger(), "Publishing feedback commands to /robot/pending_feedback");
}

FeedbackPoller::~FeedbackPoller() {
	stop_polling();
}

void FeedbackPoller::start_polling() {
	if (poll_timer_) {
		RCLCPP_WARN(this->get_logger(), "Polling already started");
		return;
	}

	poll_timer_ = this->create_wall_timer(
		std::chrono::milliseconds(poll_interval_ms_),
		[this]() { this->on_polling_tick(); });

	RCLCPP_INFO(this->get_logger(), "Feedback polling started");
}

void FeedbackPoller::stop_polling() {
	if (poll_timer_) {
		poll_timer_->cancel();
		poll_timer_ = nullptr;
	}
	RCLCPP_INFO(this->get_logger(), "Feedback polling stopped");
}

bool FeedbackPoller::set_session_id(const std::string &session_id) {
	if (session_id.empty()) {
		RCLCPP_WARN(this->get_logger(), "Cannot set empty session_id");
		return false;
	}
	std::lock_guard<std::mutex> lock(session_mutex_);
	session_id_ = session_id;
	RCLCPP_INFO(this->get_logger(), "Session ID updated to: %s", session_id_.c_str());
	return true;
}

void FeedbackPoller::set_feedback_callback(std::function<void(const json &feedback)> callback) {
	std::lock_guard<std::mutex> lock(callback_mutex_);
	feedback_callback_ = callback;
	RCLCPP_INFO(this->get_logger(), "Feedback callback registered");
}

void FeedbackPoller::set_polling_active(bool active) {
	bool was_active = polling_active_.load();
	polling_active_.store(active);

	if (!was_active && active) {
		RCLCPP_INFO(this->get_logger(), "Feedback polling activated (interaction waiting)");
	} else if (was_active && !active) {
		RCLCPP_INFO(this->get_logger(), "Feedback polling deactivated (interaction complete)");
	}
}

bool FeedbackPoller::is_polling_active() const {
	return polling_active_.load();
}

void FeedbackPoller::on_polling_tick() {
	// Skip polling if not currently waiting for interaction
	if (!polling_active_.load()) {
		RCLCPP_DEBUG(this->get_logger(), "Feedback polling not active, skipping tick");
		return;
	}

	// Get session ID safely
	std::string current_session_id;
	{
		std::lock_guard<std::mutex> lock(session_mutex_);
		current_session_id = session_id_;
	}

	if (current_session_id.empty()) {
		RCLCPP_WARN(this->get_logger(), "Cannot poll feedback: session_id not set");
		return;
	}

	RCLCPP_DEBUG(this->get_logger(), "[FEEDBACK_POLL] Polling for pending feedback on session: %s", current_session_id.c_str());

	// Build endpoint path
	std::ostringstream path_builder;
	path_builder << "/api/lessoninteraction/" << current_session_id << "/pending-feedback";
	std::string endpoint = path_builder.str();

	// Async HTTP GET request
	web_client_->sendGetAsync(
		endpoint,
		std::nullopt,
		std::vector<std::string>{},
		[this, current_session_id](const std::string &body, long http_code) {
			if (http_code == 204) {
				// No Content - no pending feedback
				RCLCPP_DEBUG(this->get_logger(), "[FEEDBACK_POLL] No pending feedback (204)");
				return;
			}

			if (http_code < 200 || http_code >= 300) {
				RCLCPP_WARN(this->get_logger(), "Failed to poll pending feedback (HTTP %ld): %s",
					http_code, body.c_str());
				return;
			}

			// Parse response JSON
			try {
				json response = json::parse(body);
				handle_pending_feedback(response, current_session_id);
			} catch (const json::exception &e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to parse feedback JSON: %s", e.what());
			}
		});
}

void FeedbackPoller::handle_pending_feedback(const json &feedback_json, const std::string &session_id) {
	try {
		// Extract feedback_id for deduplication
		if (!feedback_json.contains("feedbackId")) {
			RCLCPP_WARN(this->get_logger(), "Feedback JSON missing 'feedbackId' field");
			return;
		}

		std::string feedback_id = feedback_json["feedbackId"].get<std::string>();

		// Deduplicate: skip if this is the same feedback we already processed
		if (feedback_id == last_feedback_id_) {
			RCLCPP_DEBUG(this->get_logger(), "Skipping duplicate feedback: %s", feedback_id.c_str());
			return;
		}

		last_feedback_id_ = feedback_id;

		// Extract session ID and command
		std::string command = feedback_json.contains("feedbackCommand") 
			? feedback_json.value("feedbackCommand", "") 
			: feedback_json.value("command", "");
		int step_id = feedback_json.value("stepId", -1);

		RCLCPP_INFO(this->get_logger(),
			"Received feedback: feedbackId=%s, stepId=%d, command=%s",
			feedback_id.c_str(), step_id, command.c_str());

		// Publish feedback command to ROS topic as JSON string
		auto msg = std_msgs::msg::String();
		msg.data = feedback_json.dump();
		feedback_pub_->publish(msg);

		// Call user callback if registered
		{
			std::lock_guard<std::mutex> lock(callback_mutex_);
			if (feedback_callback_) {
				feedback_callback_(feedback_json);
			}
		}

		// Acknowledge feedback to prevent re-execution
		acknowledge_feedback(session_id, feedback_id);

	} catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Exception handling pending feedback: %s", e.what());
	}
}

void FeedbackPoller::acknowledge_feedback(const std::string &session_id, const std::string &feedback_id) {
	if (session_id.empty() || feedback_id.empty()) {
		RCLCPP_WARN(this->get_logger(), "Cannot acknowledge feedback: missing session_id or feedback_id");
		return;
	}

	// Build acknowledgment endpoint path
	std::ostringstream path_builder;
	path_builder << "/api/lessoninteraction/" << session_id
				 << "/pending-feedback/" << feedback_id
				 << "/acknowledge";
	std::string endpoint = path_builder.str();

	RCLCPP_DEBUG(this->get_logger(), "Sending acknowledgment for feedback %s", feedback_id.c_str());

	// Async HTTP PUT request (empty body)
	web_client_->sendRequestAsync(
		"PUT",
		endpoint,
		"",  // empty body
		std::nullopt,
		std::vector<std::string>{},
		[this, feedback_id](const std::string &body, long http_code) {
			if (http_code < 200 || http_code >= 300) {
				RCLCPP_WARN(this->get_logger(), "Failed to acknowledge feedback %s (HTTP %ld): %s",
					feedback_id.c_str(), http_code, body.c_str());
				return;
			}

			RCLCPP_INFO(this->get_logger(), "Successfully acknowledged feedback: %s", feedback_id.c_str());
		});
}
