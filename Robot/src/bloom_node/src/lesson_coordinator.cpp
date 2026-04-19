#include "bloom_node/lessson_coordinator.h"
#include <sstream>
#include <mutex>
#include <functional>
#include <regex>

using namespace bloom_node;

LessonCoordinator::LessonCoordinator(
    std::shared_ptr<BehaviorCoordinator> behavior_coordinator,
    std::shared_ptr<WebServiceClient> web_client,
    std::shared_ptr<StateManager> state_manager,
    const std::string &node_name
) : rclcpp::Node(node_name),
    current_step_index_(0),
    lesson_active_(false),
    current_interaction_step_(nullptr),
    waiting_for_response_(false),
    behavior_coordinator_(behavior_coordinator),
    web_client_(web_client),
    state_manager_(state_manager) {
    
    lesson_progress_publisher_ = this->create_publisher<std_msgs::msg::String>("lesson_progress", 10);
    tts_publisher_ = this->create_publisher<std_msgs::msg::String>("/tts/speak", 10);
    visual_aid_publisher_ = this->create_publisher<std_msgs::msg::String>("/face/visual_aid", 10);
    
    vosk_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/vosk/result", 10,
        std::bind(&LessonCoordinator::on_vosk_result, this, std::placeholders::_1));
    tts_done_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/tts/done", 10,
        std::bind(&LessonCoordinator::on_tts_done, this, std::placeholders::_1));

    wrap_up_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/llm/wrap_up", 10,
        std::bind(&LessonCoordinator::on_llm_wrap_up, this, std::placeholders::_1));

    llm_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/llm/mode", 10);
    llm_context_pub_ = this->create_publisher<std_msgs::msg::String>("/llm/lesson_context", 10);
    motor_pub_ = this->create_publisher<std_msgs::msg::String>("play_sequence", 10);
    stt_enable_pub_ = this->create_publisher<std_msgs::msg::String>("/stt/enable", 10);
    robot_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot/state", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            robot_state_ = msg->data;
        });
    RCLCPP_INFO(this->get_logger(), "LessonCoordinator initialized with Vosk subscriber");

    tts_interrupt_pub_ = this->create_publisher<std_msgs::msg::String>("/tts/interrupt", 10);
}

LessonCoordinator::~LessonCoordinator() {
    lesson_active_ = false;
    if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = nullptr;
    }
}

bool LessonCoordinator::load_lesson(const LessonData &lesson_data) {
    std::lock_guard<std::mutex> lock(lesson_mutex_);

    current_lesson_ = lesson_data;
    current_step_index_ = 0;
    lesson_active_ = false;

    

    RCLCPP_INFO(this->get_logger(), "Lesson loaded: %s", lesson_data.title.c_str());
    return true;
}

void LessonCoordinator::start_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);

    if (current_lesson_.sequence.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot start lesson: no steps loaded");
        return;
    }

    lesson_active_ = true;
    current_step_index_ = 0;

    RCLCPP_INFO(this->get_logger(), "[LESSON_START] Starting lesson: %s with %zu steps", current_lesson_.title.c_str(), current_lesson_.sequence.size());
    RCLCPP_INFO(this->get_logger(), "[LESSON_STATE] lesson_active_=%s, calling advance_to_next_step()", lesson_active_ ? "true" : "false");
    advance_to_next_step();
}

void LessonCoordinator::stop_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);

    if (step_timer_) {
        step_timer_->cancel();
    }

    lesson_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Lesson stopped");
}

void LessonCoordinator::reset_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);

    if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = nullptr;
    }

    lesson_active_ = false;
    current_step_index_ = 0;
    current_interaction_step_ = nullptr;
    waiting_for_tts_done_ = false;
    waiting_for_interaction_tts_ = false;
    waiting_for_wrap_up_ = false;
    waiting_for_single_turn_ = false;
    waiting_for_llm_tts_done_ = false;
    waiting_for_response_ = false;

    RCLCPP_INFO(this->get_logger(), "Lesson reset");
}

void LessonCoordinator::execute_step(const LessonStep &step) {
    waiting_for_tts_done_ = false;
    waiting_for_interaction_tts_ = false;
    waiting_for_wrap_up_ = false;
    waiting_for_single_turn_ = false;
    waiting_for_llm_tts_done_ = false;
    waiting_for_response_ = false;

    RCLCPP_INFO(this->get_logger(), "Executing step %s", step.id.c_str());
    RCLCPP_INFO(this->get_logger(), "[EXECUTE] step %s type=%s has_interaction=%s llm_follow_up=%s",
        step.id.c_str(), step.type.c_str(),
        step.has_interaction ? "true" : "false",
        step.interaction.llm_follow_up ? "true" : "false");
    auto behavior_it = step.behaviors.find("behavior");
    if (behavior_it != step.behaviors.end()) {
        const std::string &behavior_value = behavior_it->second;
        if (!behavior_value.empty() && state_manager_) {
            int timing_ms = step.timing_seconds > 0 ? (step.timing_seconds * 1000) : 0;
            state_manager_->set_state(behavior_value, timing_ms);
        }
    }

    queue_behavior(step);

    if (!step.motor_sequence.empty()) {
        auto motor_msg = std_msgs::msg::String();
        motor_msg.data = step.motor_sequence;
        motor_pub_->publish(motor_msg);
        RCLCPP_INFO(this->get_logger(), "Playing motor sequence: %s", step.motor_sequence.c_str());
    }

    // Publish visual aid or hide
    if (!step.visual_aid_images.empty()) {
        nlohmann::json va_json;
        va_json["images"] = step.visual_aid_images;
        va_json["labels"] = step.visual_aid_labels;
        if (!step.visual_aid_footers.empty()) {
            va_json["footers"] = step.visual_aid_footers;
        }
        auto va_msg = std_msgs::msg::String();
        va_msg.data = va_json.dump();
        visual_aid_publisher_->publish(va_msg);
    } else {
        auto va_msg = std_msgs::msg::String();
        va_msg.data = "{\"command\": \"hide\"}";
        visual_aid_publisher_->publish(va_msg);
    }

    if (step.interaction.llm_follow_up) {
        auto ctx_msg = std_msgs::msg::String();
        ctx_msg.data = "Lesson topic: homophones. Current question: " + step.script;
        llm_context_pub_->publish(ctx_msg);
    } else if (step.interaction.single_turn_llm) {
        // Publish the prompt template as context
        auto ctx_msg = std_msgs::msg::String();
        ctx_msg.data = step.interaction.single_turn_llm_prompt;
        llm_context_pub_->publish(ctx_msg);
    }

    if (step.has_interaction && step.interaction.wait_for_response) {
        speak_script(step.script);
        handle_interaction(step);
    } else {
        waiting_for_tts_done_ = true;
        speak_script(step.script);
    }
}

void LessonCoordinator::on_tts_done(const std_msgs::msg::String::SharedPtr) {
    RCLCPP_INFO(this->get_logger(), "[TTS_DONE_RAW] lesson_active_=%s",
        lesson_active_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "[TTS_DONE] waiting_for_tts_done_=%s | waiting_for_interaction_tts_=%s | waiting_for_wrap_up_=%s",
        waiting_for_tts_done_ ? "true" : "false",
        waiting_for_interaction_tts_ ? "true" : "false",
        waiting_for_wrap_up_ ? "true" : "false");
    if (!lesson_active_) return;

    if (waiting_for_llm_tts_done_) {
        waiting_for_llm_tts_done_ = false;
        advance_to_next_step();
        return;
    }
    if (waiting_for_interaction_tts_) {
        waiting_for_interaction_tts_ = false;

        if (current_interaction_step_ && current_interaction_step_->interaction.llm_follow_up) {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = "lesson_tangent";
            llm_mode_pub_->publish(mode_msg);
            waiting_for_wrap_up_ = true;
        } else if (current_interaction_step_ && current_interaction_step_->interaction.single_turn_llm) {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = "single_turn";
            llm_mode_pub_->publish(mode_msg);
            waiting_for_single_turn_ = true; 
        }

        // Short pause then open the response window
        step_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                step_timer_->cancel();
                step_timer_ = nullptr;
                auto stt_msg = std_msgs::msg::String();
                stt_msg.data = "true";
                stt_enable_pub_->publish(stt_msg);
                waiting_for_response_ = true;
                RCLCPP_INFO(this->get_logger(), "Now listening for student response");

                // Start the timeout timer now that we're actually listening
                if (current_interaction_step_) {
                    int timeout_seconds = current_interaction_step_->interaction.max_wait_seconds > 0 
                        ? current_interaction_step_->interaction.max_wait_seconds : 10;
                    step_timer_ = this->create_wall_timer(
                        std::chrono::seconds(timeout_seconds),
                        [this]() {
                            step_timer_->cancel();  
                            step_timer_ = nullptr;

                            if (!waiting_for_response_) return;
                            waiting_for_response_ = false;

                            // Deactivate feedback polling when interaction times out
                            if (feedback_poller_) {
                                feedback_poller_->set_polling_active(false);
                            }

                            auto stt_msg = std_msgs::msg::String();
                            stt_msg.data = "false";
                            stt_enable_pub_->publish(stt_msg);
                            const LessonStep* step = current_interaction_step_;
                            if (!step) return;

                            // Cancel wrap_up wait and reset LLM to lesson_mode
                            waiting_for_wrap_up_ = false;
                            waiting_for_single_turn_ = false;
                            auto mode_msg = std_msgs::msg::String();
                            mode_msg.data = "lesson_mode";
                            llm_mode_pub_->publish(mode_msg);

                            if (!step->interaction.fallback_visual_aid.empty()) {
                                nlohmann::json va_json;
                                va_json["images"] = step->interaction.fallback_visual_aid;
                                va_json["labels"] = step->interaction.fallback_visual_aid_labels;
                                auto va_msg = std_msgs::msg::String();
                                va_msg.data = va_json.dump();
                                visual_aid_publisher_->publish(va_msg);
                            }

                            if (!step->interaction.fallback_script.empty()) {
                                speak_script(step->interaction.fallback_script);
                            }

                            log_interaction_to_backend(step->step_order, "timeout", false);
                            waiting_for_tts_done_ = true;
                        });
                }
            });
        return;
    }

    if (waiting_for_tts_done_) {
        if (waiting_for_wrap_up_) {
            RCLCPP_WARN(this->get_logger(), "[TTS_DONE] waiting_for_wrap_up_ still true, not advancing");
            waiting_for_tts_done_ = false;
            return;
        }
        waiting_for_tts_done_ = false;
        advance_to_next_step();
    }
}

void LessonCoordinator::on_llm_wrap_up(const std_msgs::msg::String::SharedPtr) {
    RCLCPP_INFO(this->get_logger(), "[LLM_WRAP_UP] received | waiting_for_wrap_up_=%s | waiting_for_single_turn_=%s",
        waiting_for_wrap_up_ ? "true" : "false",
        waiting_for_single_turn_ ? "true" : "false");
    if (!lesson_active_) return;

    if (waiting_for_wrap_up_) {
        waiting_for_wrap_up_ = false;
        auto mode_msg = std_msgs::msg::String();
        mode_msg.data = "lesson_mode";
        llm_mode_pub_->publish(mode_msg);
        waiting_for_tts_done_ = true;
    } else if (waiting_for_single_turn_) {
        waiting_for_single_turn_ = false;
        waiting_for_llm_tts_done_ = true;
    }
}

void LessonCoordinator::queue_behavior(const LessonStep &step) {
    // Queue behaviors to the behavior coordinator
    if (!behavior_coordinator_) {
        RCLCPP_WARN(this->get_logger(), "BehaviorCoordinator not available");
        return;
    }

    for (const auto &[behavior_type, behavior_value] : step.behaviors) {
        if (!behavior_value.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Queueing behavior: %s = %s", behavior_type.c_str(), behavior_value.c_str());

            // Use request_behavior with medium priority
            behavior_coordinator_->request_behavior(behavior_value, 5, false);
        }
    }
}

void LessonCoordinator::speak_script(const std::string &script) {
    auto message = std_msgs::msg::String();
    message.data = script;
    tts_publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Speaking: %s", script.c_str());
}

void LessonCoordinator::handle_interaction(const LessonStep &step) {
    try {
        const InteractionConfig &interaction = step.interaction;

        if (!interaction.wait_for_response) {
            RCLCPP_DEBUG(this->get_logger(), "Step %s has interaction but no response required", step.id.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Handling interaction for step %s (timeout: %d seconds)",
            step.id.c_str(), interaction.max_wait_seconds);

        current_interaction_step_ = const_cast<LessonStep*>(&step);
        waiting_for_response_ = false;
        waiting_for_interaction_tts_ = true;

        if (feedback_poller_) {
            feedback_poller_->set_polling_active(true);
        }

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error handling interaction: %s", e.what());
        waiting_for_response_ = false;
        waiting_for_interaction_tts_ = false;
        if (feedback_poller_) {
            feedback_poller_->set_polling_active(false);
        }
    }
}

void LessonCoordinator::on_vosk_result(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "[VOSK] Received: '%s' | waiting_for_response_=%s | waiting_for_interaction_tts_=%s | waiting_for_wrap_up_=%s",
        msg->data.c_str(),
        waiting_for_response_ ? "true" : "false",
        waiting_for_interaction_tts_ ? "true" : "false",
        waiting_for_wrap_up_ ? "true" : "false");
    if (!msg || msg->data.empty() || !waiting_for_response_ || !current_interaction_step_) {
        return;
    }

    try {
        std::string response = msg->data;
        const LessonStep& step = *current_interaction_step_;
        const InteractionConfig& interaction = step.interaction;

        RCLCPP_DEBUG(this->get_logger(), "Received speech input: %s", response.c_str());

        // Check if response matches correct answer
        // Fuzzy match: normalize both strings and check if correct answer appears in response
        auto normalize = [](std::string s) -> std::string {
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            s.erase(std::remove_if(s.begin(), s.end(), [](char c) {
                return !std::isalnum(c) && c != ' ';
            }), s.end());
            return s;
        };
        auto digits_to_words = [](std::string s) -> std::string {
            std::regex r1("\\b1\\b"); s = std::regex_replace(s, r1, "one");
            std::regex r2("\\b2\\b"); s = std::regex_replace(s, r2, "two");
            std::regex r3("\\b3\\b"); s = std::regex_replace(s, r3, "three");
            std::regex r4("\\b4\\b"); s = std::regex_replace(s, r4, "four");
            std::regex r5("\\b5\\b"); s = std::regex_replace(s, r5, "five");
            return s;
        };
        std::string norm_response = normalize(digits_to_words(response));
        std::string norm_answer = normalize(digits_to_words(interaction.correct_answer));
        bool is_correct = !norm_answer.empty() && 
                        (norm_response.find(norm_answer) != std::string::npos);

        // Provide feedback based on correctness
        if (is_correct) {
            if (!interaction.correct_response_script.empty()) {
                speak_script(interaction.correct_response_script);
            }
            // Queue positive behavior
            if (behavior_coordinator_) {
                behavior_coordinator_->request_behavior("happy", 5, false);
            }
            RCLCPP_INFO(this->get_logger(), "Step %s: Correct response '%s'", step.id.c_str(), response.c_str());
        } else {
            if (!interaction.incorrect_response_script.empty()) {
                speak_script(interaction.incorrect_response_script);
            }
            RCLCPP_INFO(this->get_logger(), "Step %s: Incorrect response '%s' (expected '%s')",
                step.id.c_str(), response.c_str(), interaction.correct_answer.c_str());
        }

        // Log interaction result to backend
        log_interaction_to_backend(step.step_order, response, is_correct);

        waiting_for_response_ = false;
        waiting_for_interaction_tts_ = false;

        auto stt_msg = std_msgs::msg::String();
        stt_msg.data = "false";
        stt_enable_pub_->publish(stt_msg);

        if (feedback_poller_) {
            feedback_poller_->set_polling_active(false);
        }

        if (step_timer_) {
            step_timer_->cancel();
            step_timer_ = nullptr;
        }

        // Wait for feedback TTS to finish before advancing
        waiting_for_tts_done_ = true;

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing Vosk result: %s", e.what());
        waiting_for_response_ = false;

        // Deactivate feedback polling on error
        if (feedback_poller_) {
            feedback_poller_->set_polling_active(false);
        }
    }
}

void LessonCoordinator::schedule_next_step(int delay_seconds) {
    // Cancel any existing timer
    if (step_timer_) {
        step_timer_->cancel();
    }

    // Create a timer to advance to the next step after the delay
    step_timer_ = this->create_wall_timer(
        std::chrono::seconds(delay_seconds > 0 ? delay_seconds : 1),
        [this]() {
            advance_to_next_step();
        });
}

void LessonCoordinator::advance_to_next_step() {
    RCLCPP_INFO(this->get_logger(), "[ADVANCE] moving to step index %zu | tts_done=%s | interaction_tts=%s | wrap_up=%s | response=%s",
        current_step_index_,
        waiting_for_tts_done_ ? "true" : "false",
        waiting_for_interaction_tts_ ? "true" : "false",
        waiting_for_wrap_up_ ? "true" : "false",
        waiting_for_response_ ? "true" : "false");
    if (!lesson_active_) {
        return;
    }

    if (current_step_index_ >= current_lesson_.sequence.size()) {
        RCLCPP_INFO(this->get_logger(), "Lesson completed: %s", current_lesson_.lesson_id.c_str());
        lesson_active_ = false;
        update_progress_with_backend();  
        // Invoke completion callback if set
        if (completion_callback_) {
            completion_callback_(current_lesson_.lesson_id);
        }
        return;
    }

    const LessonStep &current_step = current_lesson_.sequence[current_step_index_];
    current_step_index_++;
    update_progress_with_backend();
    execute_step(current_step);
}

void LessonCoordinator::update_progress_with_backend() {
    try {
        if (!web_client_ || session_id_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot send progress: web_client or session_id missing");
            return;
        }

        // Create progress update
        LessonProgressUpdate progress;
        progress.current_step_id = current_lesson_.sequence[current_step_index_ > 0 ? current_step_index_ - 1 : 0].step_order;
        progress.completed_steps = current_step_index_;
        progress.status = lesson_active_ ? "InProgress" : "Completed";

        // Build endpoint URL
        std::string progress_endpoint = "/api/lessonsession/" + session_id_ + "/lessons/progress";

        // Send PUT request to backend
        web_client_->sendRequestAsync(
            "PUT",
            progress_endpoint,
            progress.to_json().dump(),
            std::nullopt,
            {"Content-Type: application/json"},
            [this](const std::string &body, long http_code) {
                if (http_code >= 200 && http_code < 300) {
                    RCLCPP_DEBUG(this->get_logger(), "Progress updated successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Failed to update progress (HTTP %ld): %s",
                        http_code,
                        body.c_str());
                }
            });

        RCLCPP_DEBUG(this->get_logger(), "Progress update sent to backend");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error updating progress: %s", e.what());
    }
}

void LessonCoordinator::log_interaction_to_backend(int step_order, const std::string &response, bool is_correct) {
    try {
        if (!web_client_ || session_id_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot log interaction: web_client or session_id missing");
            return;
        }

        // Create interaction log
        StudentInteractionLog interaction;
        interaction.step_id = step_order;
        interaction.interaction_type = "Response";
        interaction.student_response = response;
        interaction.is_correct = is_correct;
        interaction.response_time_ms = 0;  // TODO: Track actual response time

        // Build endpoint URL
        std::string interaction_endpoint = "/api/lessonsession/" + session_id_ + "/lessons/interactions";

        // Send POST request to backend
        web_client_->sendRequestAsync(
            "POST",
            interaction_endpoint,
            interaction.to_json().dump(),
            std::nullopt,
            {"Content-Type: application/json"},
            [this, step_order](const std::string &body, long http_code) {
                if (http_code >= 200 && http_code < 300) {
                    RCLCPP_DEBUG(this->get_logger(), "Interaction for step %d logged successfully", step_order);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Failed to log interaction for step %d (HTTP %ld): %s",
                        step_order,
                        http_code,
                        body.c_str());
                }
            });

        RCLCPP_DEBUG(this->get_logger(), "Interaction logged for step %d: response='%s', correct=%s",
            step_order, response.c_str(), is_correct ? "true" : "false");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error logging interaction: %s", e.what());
    }
}

void LessonCoordinator::skip_step() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    if (!lesson_active_) {
        RCLCPP_WARN(this->get_logger(), "skip_step called but no lesson active");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[SKIP] Skipping to next step from index %zu", current_step_index_);

    if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = nullptr;
    }

    waiting_for_tts_done_ = false;
    waiting_for_interaction_tts_ = false;
    waiting_for_wrap_up_ = false;
    waiting_for_single_turn_ = false;
    waiting_for_llm_tts_done_ = false;
    waiting_for_response_ = false;
    current_interaction_step_ = nullptr;

    auto stt_msg = std_msgs::msg::String();
    stt_msg.data = "false";
    stt_enable_pub_->publish(stt_msg);

    auto interrupt_msg = std_msgs::msg::String();
    interrupt_msg.data = "interrupt";
    tts_interrupt_pub_->publish(interrupt_msg);

    auto mode_msg = std_msgs::msg::String();
    mode_msg.data = "lesson_mode";
    llm_mode_pub_->publish(mode_msg);

    if (state_manager_) state_manager_->set_state("waiting");
    if (feedback_poller_) feedback_poller_->set_polling_active(false);

    step_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            step_timer_->cancel();
            step_timer_ = nullptr;
            advance_to_next_step();
        });
}
void LessonCoordinator::replay_step() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    if (!lesson_active_) {
        RCLCPP_WARN(this->get_logger(), "replay_step called but no lesson active");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[REPLAY] Replaying step at index %zu", current_step_index_ - 1);

    if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = nullptr;
    }

    waiting_for_tts_done_ = false;
    waiting_for_interaction_tts_ = false;
    waiting_for_wrap_up_ = false;
    waiting_for_single_turn_ = false;
    waiting_for_llm_tts_done_ = false;
    waiting_for_response_ = false;
    current_interaction_step_ = nullptr;

    auto stt_msg = std_msgs::msg::String();
    stt_msg.data = "false";
    stt_enable_pub_->publish(stt_msg);

    auto interrupt_msg = std_msgs::msg::String();
    interrupt_msg.data = "interrupt";
    tts_interrupt_pub_->publish(interrupt_msg);

    auto mode_msg = std_msgs::msg::String();
    mode_msg.data = "lesson_mode";
    llm_mode_pub_->publish(mode_msg);

    if (state_manager_) state_manager_->set_state("waiting");
    if (feedback_poller_) feedback_poller_->set_polling_active(false);

    if (current_step_index_ > 0) {
        current_step_index_--;
    }

    step_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            step_timer_->cancel();
            step_timer_ = nullptr;
            advance_to_next_step();
        });
}

void LessonCoordinator::set_step(int target_step_order) {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    if (!lesson_active_) {
        RCLCPP_WARN(this->get_logger(), "set_step called but no lesson active");
        return;
    }

    size_t target_index = current_lesson_.sequence.size();
    for (size_t i = 0; i < current_lesson_.sequence.size(); i++) {
        if (current_lesson_.sequence[i].step_order == target_step_order) {
            target_index = i;
            break;
        }
    }

    if (target_index >= current_lesson_.sequence.size()) {
        RCLCPP_WARN(this->get_logger(), "[SET_STEP] No step found with step_order=%d", target_step_order);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[SET_STEP] Jumping to step_order=%d (index=%zu)", target_step_order, target_index);

    if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = nullptr;
    }

    waiting_for_tts_done_ = false;
    waiting_for_interaction_tts_ = false;
    waiting_for_wrap_up_ = false;
    waiting_for_single_turn_ = false;
    waiting_for_llm_tts_done_ = false;
    waiting_for_response_ = false;
    current_interaction_step_ = nullptr;

    auto stt_msg = std_msgs::msg::String();
    stt_msg.data = "false";
    stt_enable_pub_->publish(stt_msg);

    auto interrupt_msg = std_msgs::msg::String();
    interrupt_msg.data = "interrupt";
    tts_interrupt_pub_->publish(interrupt_msg);

    auto mode_msg = std_msgs::msg::String();
    mode_msg.data = "lesson_mode";
    llm_mode_pub_->publish(mode_msg);

    if (state_manager_) state_manager_->set_state("waiting");
    if (feedback_poller_) feedback_poller_->set_polling_active(false);

    current_step_index_ = target_index;

    step_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            step_timer_->cancel();
            step_timer_ = nullptr;
            advance_to_next_step();
        });
}

bool LessonCoordinator::is_lesson_running() const {
    // Use const_cast to allow locking in const method
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(lesson_mutex_));
    return lesson_active_;
}

void LessonCoordinator::set_completion_callback(LessonCompletionCallback callback) {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    completion_callback_ = callback;
}

void LessonCoordinator::set_feedback_poller(std::shared_ptr<FeedbackPoller> feedback_poller) {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    feedback_poller_ = feedback_poller;
    RCLCPP_INFO(this->get_logger(), "FeedbackPoller registered with LessonCoordinator");
}

void LessonCoordinator::set_session_id(const std::string &session_id) {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    session_id_ = session_id;
    RCLCPP_INFO(this->get_logger(), "Session ID set to: %s", session_id_.c_str());
}

std::string LessonCoordinator::get_session_id() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(lesson_mutex_));
    return session_id_;
}


