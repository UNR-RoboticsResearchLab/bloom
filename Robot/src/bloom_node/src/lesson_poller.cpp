#include "bloom_node/lesson_poller.h"
#include <rclcpp/logging.hpp>
#include <sstream>

using namespace bloom_node;
using namespace std::chrono_literals;

LessonPoller::LessonPoller(
	std::shared_ptr<WebServiceClient> web_client,
	std::shared_ptr<LessonCoordinator> lesson_coord,
	const std::string &session_id,
	int poll_interval_ms,
	const std::string &node_name
)
	: rclcpp::Node(node_name),
	  web_client_(web_client),
	  lesson_coord_(lesson_coord),
	  session_id_(session_id),
	  poll_interval_ms_(poll_interval_ms) {
	RCLCPP_INFO(this->get_logger(), "LessonPoller initialized (session_id: %s, poll_interval: %dms)",
		session_id_.c_str(), poll_interval_ms_);

	// Subscribe to /load_lesson topic for direct (backendless) lesson loading
	load_lesson_sub_ = this->create_subscription<std_msgs::msg::String>(
		"/load_lesson", 10,
		[this](const std_msgs::msg::String::SharedPtr msg) {
			if (!msg || msg->data.empty()) return;
			RCLCPP_INFO(this->get_logger(), "Received lesson JSON on /load_lesson topic");
			try {
				auto lesson_json = json::parse(msg->data);
				handle_pending_lesson(lesson_json);
			} catch (const json::exception &e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to parse /load_lesson JSON: %s", e.what());
			}
		});

	RCLCPP_INFO(this->get_logger(), "Subscribed to /load_lesson for backendless lesson loading");
}

LessonPoller::~LessonPoller() {
	stop_polling();
}

void LessonPoller::start_polling() {
	if (poll_timer_) {
		RCLCPP_WARN(this->get_logger(), "Polling already started");
		return;
	}

	poll_timer_ = this->create_wall_timer(
		std::chrono::milliseconds(poll_interval_ms_),
		[this]() { this->on_polling_tick(); });

	RCLCPP_INFO(this->get_logger(), "Lesson polling started");
}

void LessonPoller::stop_polling() {
	if (poll_timer_) {
		poll_timer_->cancel();
		poll_timer_ = nullptr;
	}
	RCLCPP_INFO(this->get_logger(), "Lesson polling stopped");
}

bool LessonPoller::set_session_id(const std::string &session_id) {
	if (session_id.empty()) {
		RCLCPP_WARN(this->get_logger(), "Cannot set empty session_id");
		return false;
	}
	std::lock_guard<std::mutex> lock(session_mutex_);
	session_id_ = session_id;
	RCLCPP_INFO(this->get_logger(), "Session ID updated to: %s", session_id_.c_str());
	return true;
}

void LessonPoller::on_polling_tick() {
	// Skip polling if a lesson is currently executing
	RCLCPP_DEBUG(this->get_logger(), "polling tick...");
	if (currently_executing_.load()) {
		RCLCPP_DEBUG(this->get_logger(), "Skipping poll: lesson currently executing");
		return;
	}

	// Get session ID safely
	std::string current_session_id;
	{
		std::lock_guard<std::mutex> lock(session_mutex_);
		current_session_id = session_id_;
	}

	if (current_session_id.empty()) {
		RCLCPP_DEBUG(this->get_logger(), "Cannot poll: session_id not set");
		return;
	}

	// Build endpoint path
	std::ostringstream path_builder;
	path_builder << "/api/robotsessions/" << current_session_id << "/pending-lesson";
	std::string endpoint = path_builder.str();


	// Async HTTP GET request
	web_client_->sendGetAsync(
		endpoint,
		std::nullopt,
		std::vector<std::string>{},
		[this, endpoint](const std::string &body, long http_code) {
			if (http_code == 204) {
				// No Content - no pending lesson
				RCLCPP_DEBUG(this->get_logger(), "No pending lesson");
				return;
			}

			if (http_code < 200 || http_code >= 300) {
				RCLCPP_WARN(this->get_logger(), "Failed to poll pending lesson (HTTP %ld): %s",
					http_code, body.c_str());
				return;
			}

			// Parse response JSON
			try {
				json response = json::parse(body);
				if (!response.contains("hasPendingLesson") || !response["hasPendingLesson"].get<bool>()) {
					RCLCPP_DEBUG(this->get_logger(), "Response indicates no pending lesson");
					return;
				}

				if (!response.contains("lesson")) {
					RCLCPP_WARN(this->get_logger(), "Response missing 'lesson' field");
					return;
				}

				handle_pending_lesson(response["lesson"]);
			} catch (const json::exception &e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to parse lesson JSON: %s", e.what());
			}
		});
}

void LessonPoller::handle_pending_lesson(const json &lesson_json) {
	try {


		// Extract lesson_id for deduplication
		if (!lesson_json.contains("id")) {
			RCLCPP_WARN(this->get_logger(), "Lesson JSON missing 'id' field");
			return;
		}

		std::string lesson_id = lesson_json["id"].get<std::string>();
		// Deduplicate: skip if this is the same lesson we just ran
		if (lesson_id == last_lesson_id_) {
			RCLCPP_DEBUG(this->get_logger(), "Skipping duplicate lesson: %s", lesson_id.c_str());
			return;
		}

		// Mark as executing before loading to prevent race conditions
		currently_executing_.store(true);
		last_lesson_id_ = lesson_id;

		// Parse lesson JSON to LessonData struct
		LessonData lesson_data;
		lesson_data.lesson_id = lesson_id;

		if (lesson_json.contains("title")) {
			lesson_data.title = lesson_json["title"].get<std::string>();
		}

		if (lesson_json.contains("learning_objectives")) {
			lesson_data.learning_objectives = lesson_json["learning_objectives"].get<std::vector<std::string>>();
		}

		// Parse sequence steps
		if (lesson_json.contains("sequence") && lesson_json["sequence"].is_array()) {
			for (const auto &step_json : lesson_json["sequence"]) {
				LessonStep step;
				step.id = step_json.value("id", 0);
				step.type = step_json.value("type", "");
				step.script = step_json.value("script", "");
				step.timing_seconds = step_json.value("timing_seconds", 0);

				// Parse behaviors map
				if (step_json.contains("behaviors") && step_json["behaviors"].is_object()) {
					for (auto &[key, value] : step_json["behaviors"].items()) {
						step.behaviors[key] = value.get<std::string>();
					}
				}
				step.motor_sequence = step_json.value("motor_sequence", "");
				// Parse visual aid
				if (step_json.contains("visual_aid")) {
					if (step_json["visual_aid"].is_array()) {
						for (const auto &img : step_json["visual_aid"]) {
							step.visual_aid_images.push_back(img.get<std::string>());
						}
					} else if (step_json["visual_aid"].is_string()) {
						step.visual_aid_images.push_back(step_json["visual_aid"].get<std::string>());
					}
				}
				if (step_json.contains("visual_aid_labels") && step_json["visual_aid_labels"].is_array()) {
					for (const auto &lbl : step_json["visual_aid_labels"]) {
						step.visual_aid_labels.push_back(lbl.get<std::string>());
					}
				}
				if (step_json.contains("visual_aid_footers") && step_json["visual_aid_footers"].is_array()) {
					for (const auto &ftr : step_json["visual_aid_footers"]) {
						step.visual_aid_footers.push_back(ftr.get<std::string>());
					}
				}
				// Parse interaction config if present
				// has_interaction is true if an interaction block exists, even without explicit flag
				step.has_interaction = step_json.contains("interaction") && step_json["interaction"].is_object();
				if (step.has_interaction) {
					const auto &interaction_json = step_json["interaction"];
					step.interaction.wait_for_response = interaction_json.value("wait_for_response", false);
					step.interaction.max_wait_seconds = interaction_json.value("max_wait_seconds", 10);
					step.interaction.correct_answer = interaction_json.value("correct_answer", "");
					step.interaction.correct_response_script = interaction_json.value("correct_response_script", "");
					step.interaction.incorrect_response_script = interaction_json.value("incorrect_response_script", "");
					step.interaction.fallback_script = interaction_json.value("fallback_script", "");
					step.interaction.llm_follow_up = interaction_json.value("llm_follow_up", false);
				}

				lesson_data.sequence.push_back(step);
			}
		}

		RCLCPP_INFO(this->get_logger(), "Loading lesson: %s (%zu steps)",
			lesson_data.title.c_str(), lesson_data.sequence.size());

		// Load and start the lesson
		bool loaded = lesson_coord_->load_lesson(lesson_data);
		if (!loaded) {
			RCLCPP_ERROR(this->get_logger(), "Failed to load lesson: %s", lesson_id.c_str());
			currently_executing_.store(false);
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Starting lesson: %s", lesson_id.c_str());

		// Set completion callback to clear currently_executing_ flag when lesson finishes
		lesson_coord_->set_completion_callback(
			[this](const std::string &completed_lesson_id) {
				RCLCPP_INFO(this->get_logger(), "Lesson completion callback: %s", completed_lesson_id.c_str());
				currently_executing_.store(false);
			});

		lesson_coord_->start_lesson();

	} catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Exception handling pending lesson: %s", e.what());
		currently_executing_.store(false);
	}
}
