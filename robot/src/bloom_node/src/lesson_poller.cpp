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
	: rclcpp::Node(node_name, rclcpp::NodeOptions().use_global_arguments(false)),
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
					if (currently_executing_.load() && paired_.load()) {
						bool has_active_lesson = j.contains("activeLessonId") && !j["activeLessonId"].is_null();
						if (!has_active_lesson) {
							RCLCPP_WARN(this->get_logger(), "[SAFETY] Lesson running but backend reports no active lesson — stopping");
							if (lesson_coord_) lesson_coord_->stop_lesson();
							last_lesson_id_ = "";
							currently_executing_.store(false);
							stop_step_control_polling();
							std::string ack_endpoint = "/api/lessonsession/" + current_session_id + "/lessons/step-control";
							web_client_->sendRequestAsync("DELETE", ack_endpoint, std::nullopt, std::nullopt, {}, 
								[this](const std::string&, long){});
						}
					}
					if (!currently_executing_.load() && !last_lesson_id_.empty() && paired_.load()) {
						bool has_active_lesson = j.contains("activeLessonId") && !j["activeLessonId"].is_null();
						if (!has_active_lesson) {
							RCLCPP_INFO(this->get_logger(), "[POLLER] Backend cleared active lesson — ready for new lesson");
							last_lesson_id_ = "";
						}
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
        RCLCPP_DEBUG(this->get_logger(), "Skipping lesson poll: lesson currently executing");
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
                RCLCPP_DEBUG(this->get_logger(), "Pending lesson response: %s", response.dump().c_str());

                if (!response.contains("hasPendingLesson") || !response["hasPendingLesson"].get<bool>()) {
                    RCLCPP_INFO(this->get_logger(), "Response indicates no pending lesson");
                    return;
                }

                if (!response.contains("lesson")) {
                    RCLCPP_WARN(this->get_logger(), "Response missing 'lesson' field");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Pending lesson received, calling handle_pending_lesson");
				std::string lesson_run_id = "";
				if (response.contains("activeLessonRunId") && !response["activeLessonRunId"].is_null()) {
					lesson_run_id = response["activeLessonRunId"].get<std::string>();
					RCLCPP_INFO(this->get_logger(), "Lesson run ID: %s", lesson_run_id.c_str());
				}
                handle_pending_lesson(response["lesson"], lesson_run_id);
            } catch (const json::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse lesson JSON: %s", e.what());
            }
        });
}

void LessonPoller::handle_pending_lesson(const json &lesson_json, const std::string &lesson_run_id) {
	try {


		// Extract lesson_id for deduplication
		if (!lesson_json.contains("id")) {
			RCLCPP_WARN(this->get_logger(), "Lesson JSON missing 'id' field");
			return;
		}

		std::string lesson_id = lesson_json["id"].get<std::string>();
		if (!lesson_run_id.empty() && lesson_run_id == last_lesson_id_) {
			RCLCPP_DEBUG(this->get_logger(), "Skipping duplicate lesson: %s", lesson_id.c_str());
			return;
		}

		// Mark as executing before loading to prevent race conditions
		currently_executing_.store(true);
		last_lesson_id_ = lesson_run_id.empty() ? lesson_id : lesson_run_id;

		// Parse lesson JSON to LessonData struct
		LessonData lesson_data;
		lesson_data.lesson_id = lesson_id;
		lesson_data.lesson_run_id = lesson_run_id;

		if (lesson_json.contains("title")) {
			lesson_data.title = lesson_json["title"].get<std::string>();
		}
		lesson_data.conversation_mode = (lesson_data.title == "Conversation Mode");

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
				step.timing_seconds = (step_json.contains("timingSeconds") && !step_json["timingSeconds"].is_null()) 
    				? step_json["timingSeconds"].get<int>() : 0;

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
				// Parse visual aid labels
				if (step_json.contains("visualAidLabels") && !step_json["visualAidLabels"].is_null()) {
					try {
						auto labels_str = step_json["visualAidLabels"].get<std::string>();
						auto labels_json = json::parse(labels_str);
						if (labels_json.is_array()) {
							for (const auto &label : labels_json) {
								step.visual_aid_labels.push_back(label.get<std::string>());
							}
						}
					} catch (...) {
						RCLCPP_WARN(this->get_logger(), "Failed to parse visualAidLabels for step %s", step.id.c_str());
					}
				}
				// Parse visual aid footers
				if (step_json.contains("visualAidFooters") && !step_json["visualAidFooters"].is_null()) {
					try {
						auto footers_str = step_json["visualAidFooters"].get<std::string>();
						auto footers_json = json::parse(footers_str);
						if (footers_json.is_array()) {
							for (const auto &footer : footers_json) {
								step.visual_aid_footers.push_back(footer.get<std::string>());
							}
						}
					} catch (...) {
						RCLCPP_WARN(this->get_logger(), "Failed to parse visualAidFooters for step %s", step.id.c_str());
					}
				}
				// Parse motor sequence
				if (step_json.contains("motorSequence") && !step_json["motorSequence"].is_null()) {
					step.motor_sequence = step_json["motorSequence"].get<std::string>();
				}
				step.has_interaction = false;
				if (step_json.contains("interaction") && !step_json["interaction"].is_null()) {
					json interaction_json;
					bool got_interaction = false;

					if (step_json["interaction"].is_object()) {
						interaction_json = step_json["interaction"];
						got_interaction = true;
					} else if (step_json["interaction"].is_string()) {
						std::string interaction_str = step_json["interaction"].get<std::string>();
						if (!interaction_str.empty()) {
							try {
								interaction_json = json::parse(interaction_str);
								got_interaction = true;
							} catch (...) {
								RCLCPP_WARN(this->get_logger(), "Failed to parse interaction string for step %s", step.id.c_str());
							}
						}
					}

					if (got_interaction) {
						try {
							step.has_interaction = true;
							step.interaction.wait_for_response = (interaction_json.contains("waitForResponse") && !interaction_json["waitForResponse"].is_null())
								? interaction_json["waitForResponse"].get<bool>() : false;
							step.interaction.llm_follow_up = (interaction_json.contains("llmFollowUp") && !interaction_json["llmFollowUp"].is_null())
								? interaction_json["llmFollowUp"].get<bool>() : false;
							step.interaction.single_turn_llm = (interaction_json.contains("singleTurnLlm") && !interaction_json["singleTurnLlm"].is_null())
								? interaction_json["singleTurnLlm"].get<bool>() : false;
							step.interaction.max_wait_seconds = (interaction_json.contains("maxWaitSeconds") && !interaction_json["maxWaitSeconds"].is_null())
								? interaction_json["maxWaitSeconds"].get<int>() : 0;
							step.interaction.correct_answer = (interaction_json.contains("correctAnswer") && !interaction_json["correctAnswer"].is_null())
								? interaction_json["correctAnswer"].get<std::string>() : "";
							step.interaction.correct_response_script = (interaction_json.contains("correctResponseScript") && !interaction_json["correctResponseScript"].is_null())
								? interaction_json["correctResponseScript"].get<std::string>() : "";
							step.interaction.incorrect_response_script = (interaction_json.contains("incorrectResponseScript") && !interaction_json["incorrectResponseScript"].is_null())
								? interaction_json["incorrectResponseScript"].get<std::string>() : "";
							step.interaction.fallback_script = (interaction_json.contains("fallbackScript") && !interaction_json["fallbackScript"].is_null())
								? interaction_json["fallbackScript"].get<std::string>() : "";
							step.interaction.single_turn_llm_prompt = (interaction_json.contains("singleTurnLlmPrompt") && !interaction_json["singleTurnLlmPrompt"].is_null())
								? interaction_json["singleTurnLlmPrompt"].get<std::string>() : "";
						} catch (const std::exception &e) {
							RCLCPP_WARN(this->get_logger(), "Failed to extract interaction fields for step %s: %s", 
								step.id.c_str(), e.what());
							step.has_interaction = false;
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
				stop_step_control_polling();
				currently_executing_.store(false);
			});

		start_step_control_polling();
		lesson_coord_->start_lesson();
		RCLCPP_INFO(this->get_logger(), "[DONE] start_lesson() completed");

	} catch (const std::exception &e) {
		RCLCPP_ERROR(this->get_logger(), "Exception handling pending lesson: %s", e.what());
		currently_executing_.store(false);
	}

	
}

void LessonPoller::start_step_control_polling() {
    if (step_control_timer_) return;
    step_control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1500),
        [this]() { this->on_step_control_tick(); });
    RCLCPP_INFO(this->get_logger(), "Step control polling started");
}

void LessonPoller::stop_step_control_polling() {
    if (step_control_timer_) {
        step_control_timer_->cancel();
        step_control_timer_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Step control polling stopped");
}

void LessonPoller::on_step_control_tick() {
    if (!currently_executing_.load()) {
        stop_step_control_polling();
        return;
    }

    std::string current_session_id;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        current_session_id = session_id_;
    }

    if (current_session_id.empty()) return;

    std::string endpoint = "/api/lessonsession/" + current_session_id + "/lessons/step-control";

    web_client_->sendGetAsync(
        endpoint,
        std::nullopt,
        std::vector<std::string>{},
        [this, current_session_id, endpoint](const std::string &body, long http_code) {
            if (http_code == 204) return; // No command pending

            if (http_code < 200 || http_code >= 300) {
                RCLCPP_WARN(this->get_logger(), "Step control poll failed (HTTP %ld)", http_code);
                return;
            }

            try {
                auto j = json::parse(body);
                std::string command = j.value("command", "");
                RCLCPP_INFO(this->get_logger(), "[STEP_CONTROL] Received command: %s", command.c_str());

                if (command == "skip") {
                    if (lesson_coord_) lesson_coord_->skip_step();
                } else if (command == "replay") {
                    if (lesson_coord_) lesson_coord_->replay_step();
                } else if (command == "set_step") {
                    int target = j.value("targetStep", -1);
                    if (target >= 0 && lesson_coord_) {
                        lesson_coord_->set_step(target);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[STEP_CONTROL] set_step missing or invalid targetStep");
                    }
				} else if (command == "stop") {
					RCLCPP_INFO(this->get_logger(), "[STEP_CONTROL] Stop command received - stopping lesson");
					if (lesson_coord_) lesson_coord_->stop_lesson();
					currently_executing_.store(false);
					stop_step_control_polling();
                } else {
                    RCLCPP_WARN(this->get_logger(), "[STEP_CONTROL] Unknown command: %s", command.c_str());
                }

                // Acknowledge
                std::string ack_endpoint = "/api/lessonsession/" + current_session_id + "/lessons/step-control";
                web_client_->sendRequestAsync(
                    "DELETE",
                    ack_endpoint,
                    std::nullopt,
                    std::nullopt,
                    std::vector<std::string>{},
                    [this](const std::string &, long ack_code) {
                        if (ack_code >= 200 && ack_code < 300) {
                            RCLCPP_INFO(this->get_logger(), "[STEP_CONTROL] Command acknowledged");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "[STEP_CONTROL] Failed to acknowledge command (HTTP %ld)", ack_code);
                        }
                    });

            } catch (const json::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse step control response: %s", e.what());
            }
        });
}

void LessonPoller::set_pairing_code(const std::string &pairing_code) {
		std::lock_guard<std::mutex> lock(session_mutex_);
		pairing_code_ = pairing_code;
	}
