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
	session_code_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/session_code", 10);

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
    RCLCPP_DEBUG(this->get_logger(), "polling tick...");

    // Get session ID
    std::string current_session_id;
    std::string current_pairing_code;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        current_session_id = session_id_;
        current_pairing_code = pairing_code_;
    }

    if (current_session_id.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Cannot poll: session_id not set");
        return;
    }

    // Session status check (pairing)
    // Always check session status to detect pairing and disconnection
    std::ostringstream status_path;
    status_path << "/api/robotsession/" << current_session_id;

    web_client_->sendGetAsync(
        status_path.str(),
        std::nullopt,
        std::vector<std::string>{},
        [this, current_pairing_code, current_session_id](const std::string &body, long http_code) {
            auto msg = std_msgs::msg::String();

            if (http_code == 200) {
                try {
                    auto j = json::parse(body);
                    bool has_user = j.contains("userId") && !j["userId"].is_null();

                    if (has_user && !paired_.load()) {
                        // Just got paired - clear the face and update LessonCoordinator with session ID
                        paired_.store(true);
                        msg.data = "";
                        session_code_pub_->publish(msg);

                        // Propagate session_id to LessonCoordinator now that user is set
                        if (lesson_coord_) {
                            lesson_coord_->set_session_id(current_session_id);
                            RCLCPP_INFO(this->get_logger(), "Updated LessonCoordinator with session ID: %s", current_session_id.c_str());
                        }

                        RCLCPP_INFO(this->get_logger(), "Session paired — clearing pairing code from face");
                    } else if (!has_user && paired_.load()) {
                        // Lost pairing - show code again
                        paired_.store(false);
                        msg.data = current_pairing_code;
                        session_code_pub_->publish(msg);
                        RCLCPP_WARN(this->get_logger(), "Session lost pairing — showing pairing code again");
                    } else if (!has_user && !paired_.load()) {
                        // Still waiting - keep republishing code so face_node has it
                        msg.data = current_pairing_code;
                        session_code_pub_->publish(msg);
                    }
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse session status response");
                }
            } else {
                // Backend unreachable or session gone - show code again
                if (paired_.load()) {
                    paired_.store(false);
                    RCLCPP_WARN(this->get_logger(), "Backend unreachable (HTTP %ld) — showing pairing code again", http_code);
                }
                msg.data = current_pairing_code;
                session_code_pub_->publish(msg);
            }
        });

    //Lesson polling
    if (currently_executing_.load()) {
        RCLCPP_WARN(this->get_logger(), "Skipping lesson poll: lesson currently executing (may be stuck)");
        return;
    }

    std::ostringstream path_builder;
    path_builder << "/api/lessonsession/" << current_session_id << "/pending-lesson";
    std::string endpoint = path_builder.str();

    web_client_->sendGetAsync(
        endpoint,
        std::nullopt,
        std::vector<std::string>{},
        [this, endpoint, lesson_coord = lesson_coord_](const std::string &body, long http_code) {
            if (http_code == 204) {
                RCLCPP_INFO(this->get_logger(), "No pending lesson");
                return;
            }

            if (http_code < 200 || http_code >= 300) {
                RCLCPP_INFO(this->get_logger(), "Failed to poll pending lesson (HTTP %ld): %s",
                    http_code, body.c_str());
				RCLCPP_INFO(this->get_logger(), "Endpoint was: %s", endpoint.c_str());
                return;
            }

            try {
                json response = json::parse(body);
                RCLCPP_INFO(this->get_logger(), "Pending lesson response: %s", response.dump().c_str());

                if (!response.contains("hasPendingLesson") || !response["hasPendingLesson"].get<bool>()) {
                    RCLCPP_INFO(this->get_logger(), "Response indicates no pending lesson");
                    return;
                }

                if (!response.contains("lesson")) {
                    RCLCPP_WARN(this->get_logger(), "Response missing 'lesson' field");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Pending lesson received, calling handle_pending_lesson");
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
		if (lesson_json.contains("steps") && lesson_json["steps"].is_array()) {
			for (const auto &step_json : lesson_json["steps"]) {
				LessonStep step;
				step.id = step_json.value("id", "");
				step.step_order = step_json.value("stepOrder", 0);
				step.type = step_json.value("type", "");
				step.script = step_json.value("script", "");
				step.timing_seconds = step_json.value("timingSeconds", 0);

				if (step_json.contains("behaviors") && step_json["behaviors"].is_string()) {
					std::string behaviors_str = step_json["behaviors"].get<std::string>();
					if (!behaviors_str.empty()) {
						try {
							auto behaviors_json = json::parse(behaviors_str);
							if (behaviors_json.is_object()) {
								for (auto &[key, value] : behaviors_json.items()) {
									if (value.is_string()) {
										step.behaviors[key] = value.get<std::string>();
									}
								}
							}
						} catch (...) {
							RCLCPP_WARN(this->get_logger(), "Failed to parse behaviors JSON string for step %s", step.id.c_str());
						}
					}
				}
				if (step_json.contains("visualAid") && !step_json["visualAid"].is_null()) {
					if (step_json["visualAid"].is_string()) {
						std::string va_str = step_json["visualAid"].get<std::string>();
						if (!va_str.empty()) {
							try {
								auto va_json = json::parse(va_str);
								if (va_json.is_array()) {
									for (const auto &img : va_json) {
										step.visual_aid_images.push_back(img.get<std::string>());
									}
								}
							} catch (...) {
								step.visual_aid_images.push_back(va_str);
							}
						}
					}
				}
				step.has_interaction = false;
				if (step_json.contains("interaction") && !step_json["interaction"].is_null()) {
					if (step_json["interaction"].is_string()) {
						std::string interaction_str = step_json["interaction"].get<std::string>();
						if (!interaction_str.empty()) {
							try {
								auto interaction_json = json::parse(interaction_str);
								step.has_interaction = true;
								step.interaction.wait_for_response = interaction_json.value("wait_for_response", false);
								step.interaction.max_wait_seconds = interaction_json.value("max_wait_seconds", 10);
								step.interaction.correct_answer = interaction_json.value("correct_answer", "");
								step.interaction.correct_response_script = interaction_json.value("correct_response_script", "");
								step.interaction.incorrect_response_script = interaction_json.value("incorrect_response_script", "");
								step.interaction.fallback_script = interaction_json.value("fallback_script", "");
								step.interaction.llm_follow_up = interaction_json.value("llm_follow_up", false);
								step.interaction.single_turn_llm = interaction_json.value("single_turn_llm", false);
								step.interaction.single_turn_llm_prompt = interaction_json.value("single_turn_llm_prompt", "");
							} catch (...) {
								RCLCPP_WARN(this->get_logger(), "Failed to parse interaction JSON string for step %s", step.id.c_str());
								step.has_interaction = false;
							}
						}
					}
				}

				lesson_data.sequence.push_back(step);
				RCLCPP_INFO(this->get_logger(), 
					"[PARSE] Step %s order=%d type=%s script='%.50s' has_interaction=%s behaviors=%zu visual_aids=%zu",
					step.id.c_str(),
					step.step_order,
					step.type.c_str(),
					step.script.c_str(),
					step.has_interaction ? "true" : "false",
					step.behaviors.size(),
					step.visual_aid_images.size());
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

		RCLCPP_INFO(this->get_logger(), "[SUCCESS] Lesson loaded: %s", lesson_id.c_str());
		RCLCPP_INFO(this->get_logger(), "[STARTING] Calling start_lesson() on LessonCoordinator");

		// Set completion callback to clear currently_executing_ flag when lesson finishes
		lesson_coord_->set_completion_callback(
			[this](const std::string &completed_lesson_id) {
				RCLCPP_INFO(this->get_logger(), "Lesson completion callback: %s", completed_lesson_id.c_str());
				currently_executing_.store(false);
			});

		lesson_coord_->start_lesson();
		RCLCPP_INFO(this->get_logger(), "[DONE] start_lesson() completed");

	} catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Exception handling pending lesson: %s", e.what());
		currently_executing_.store(false);
	}

	
}

void LessonPoller::set_pairing_code(const std::string &pairing_code) {
		std::lock_guard<std::mutex> lock(session_mutex_);
		pairing_code_ = pairing_code;
	}
