#include "bloom_node/lessson_coordinator.h"
#include <sstream>
#include <mutex>
#include <functional>

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

    RCLCPP_INFO(this->get_logger(), "LessonCoordinator initialized with Vosk subscriber");
}

LessonCoordinator::~LessonCoordinator() {
    stop_lesson();
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

    RCLCPP_INFO(this->get_logger(), "Starting lesson: %s", current_lesson_.title.c_str());
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

    stop_lesson();
    current_step_index_ = 0;

    RCLCPP_INFO(this->get_logger(), "Lesson reset");
}

void LessonCoordinator::execute_step(const LessonStep &step) {
    RCLCPP_INFO(this->get_logger(), "Executing step %d", step.id);

    // Set robot state based on the 'behavior' field from the lesson step
    auto behavior_it = step.behaviors.find("behavior");
    if (behavior_it != step.behaviors.end()) {
        const std::string &behavior_value = behavior_it->second;
        if (!behavior_value.empty() && state_manager_) {
            // Convert timing_seconds to milliseconds and pass to set_state
            int timing_ms = step.timing_seconds > 0 ? (step.timing_seconds * 1000) : 0;
            state_manager_->set_state(behavior_value, timing_ms);
            RCLCPP_INFO(this->get_logger(), "Set state to: %s (timing: %d ms)", behavior_value.c_str(), timing_ms);
        }
    }

    queue_behavior(step);
    speak_script(step.script);

    if (!step.visual_aid_url.empty()) {
        auto va_msg = std_msgs::msg::String();
        va_msg.data = "{\"images\": [\"" + step.visual_aid_url + "\"], \"labels\": [\"\"]}";
        visual_aid_publisher_->publish(va_msg);
        RCLCPP_INFO(this->get_logger(), "Showing visual aid: %s", step.visual_aid_url.c_str());
    } else {
        auto va_msg = std_msgs::msg::String();
        va_msg.data = "{\"command\": \"hide\"}";
        visual_aid_publisher_->publish(va_msg);
    }

    if (step.has_interaction) {
        handle_interaction(step);
    } else {
        // For non-interactive steps, schedule the next step after the timing duration
        schedule_next_step(step.timing_seconds);
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
            RCLCPP_DEBUG(this->get_logger(), "Step %d has no response required", step.id);
            return;
        }

        int timeout_seconds = interaction.max_wait_seconds > 0 ? interaction.max_wait_seconds : 10;

        RCLCPP_INFO(this->get_logger(),
            "Handling interaction for step %d (timeout: %d seconds, correct_answer: %s)",
            step.id,
            timeout_seconds,
            interaction.correct_answer.c_str());

        // Store current step for vosk callback to access
        current_interaction_step_ = const_cast<LessonStep*>(&step);
        waiting_for_response_ = true;

        // Activate feedback polling while waiting for interaction
        if (feedback_poller_) {
            feedback_poller_->set_polling_active(true);
            RCLCPP_DEBUG(this->get_logger(), "Activated feedback polling for interaction on step %d", step.id);
        }

        // Cancel any existing timer
        if (step_timer_) {
            step_timer_->cancel();
        }

        // Set timeout timer as fallback
        step_timer_ = this->create_wall_timer(
            std::chrono::seconds(timeout_seconds),
            [this, step]() {
                if (!waiting_for_response_) return;

                RCLCPP_WARN(this->get_logger(), "Step %d interaction timeout - using fallback", step.id);
                waiting_for_response_ = false;

                // Deactivate feedback polling
                if (feedback_poller_) {
                    feedback_poller_->set_polling_active(false);
                }

                // Use fallback script if provided
                if (!step.interaction.fallback_script.empty()) {
                    speak_script(step.interaction.fallback_script);
                }

                // Log timeout interaction
                log_interaction_to_backend(step.id, "timeout", false);

                // Move to next step
                advance_to_next_step();
            });
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error handling interaction: %s", e.what());
        waiting_for_response_ = false;

        // Deactivate feedback polling on error
        if (feedback_poller_) {
            feedback_poller_->set_polling_active(false);
        }
    }
}

void LessonCoordinator::on_vosk_result(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg || msg->data.empty() || !waiting_for_response_ || !current_interaction_step_) {
        return;
    }

    try {
        std::string response = msg->data;
        const LessonStep& step = *current_interaction_step_;
        const InteractionConfig& interaction = step.interaction;

        RCLCPP_DEBUG(this->get_logger(), "Received speech input: %s", response.c_str());

        // Check if response matches correct answer
        bool is_correct = (response == interaction.correct_answer);

        // Provide feedback based on correctness
        if (is_correct) {
            if (!interaction.correct_response_script.empty()) {
                speak_script(interaction.correct_response_script);
            }
            // Queue positive behavior
            if (behavior_coordinator_) {
                behavior_coordinator_->request_behavior("happy", 5, false);
            }
            RCLCPP_INFO(this->get_logger(), "Step %d: Correct response '%s'", step.id, response.c_str());
        } else {
            if (!interaction.incorrect_response_script.empty()) {
                speak_script(interaction.incorrect_response_script);
            }
            RCLCPP_INFO(this->get_logger(), "Step %d: Incorrect response '%s' (expected '%s')",
                step.id, response.c_str(), interaction.correct_answer.c_str());
        }

        // Log interaction result to backend
        log_interaction_to_backend(step.id, response, is_correct);

        // Mark as no longer waiting and cancel timeout
        waiting_for_response_ = false;

        // Deactivate feedback polling
        if (feedback_poller_) {
            feedback_poller_->set_polling_active(false);
        }

        if (step_timer_) {
            step_timer_->cancel();
            step_timer_ = nullptr;
        }

        // Move to next step
        advance_to_next_step();
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
    if (!lesson_active_) {
        return;
    }

    if (current_step_index_ >= current_lesson_.sequence.size()) {
        RCLCPP_INFO(this->get_logger(), "Lesson completed: %s", current_lesson_.lesson_id.c_str());
        lesson_active_ = false;

        // Invoke completion callback if set
        if (completion_callback_) {
            completion_callback_(current_lesson_.lesson_id);
        }
        return;
    }

    const LessonStep &current_step = current_lesson_.sequence[current_step_index_];
    execute_step(current_step);

    current_step_index_++;
}

void LessonCoordinator::update_progress_with_backend() {
    try {
        if (!web_client_ || session_id_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot send progress: web_client or session_id missing");
            return;
        }

        // Create progress update
        LessonProgressUpdate progress;
        progress.current_step_id = current_step_index_;
        progress.completed_steps = current_step_index_;
        progress.status = lesson_active_ ? "InProgress" : "Completed";

        // Build endpoint URL
        std::string progress_endpoint = "/api/robotsessions/" + session_id_ + "lessons/progress";

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

void LessonCoordinator::log_interaction_to_backend(int step_id, const std::string &response, bool is_correct) {
    try {
        if (!web_client_ || session_id_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot log interaction: web_client or session_id missing");
            return;
        }

        // Create interaction log
        StudentInteractionLog interaction;
        interaction.step_id = step_id;
        interaction.interaction_type = "Response";
        interaction.student_response = response;
        interaction.is_correct = is_correct;
        interaction.response_time_ms = 0;  // TODO: Track actual response time

        // Build endpoint URL
        std::string interaction_endpoint = "/api/robotsessions/" + session_id_ + "/lessons/interactions";

        // Send POST request to backend
        web_client_->sendRequestAsync(
            "POST",
            interaction_endpoint,
            interaction.to_json().dump(),
            std::nullopt,
            {"Content-Type: application/json"},
            [this, step_id](const std::string &body, long http_code) {
                if (http_code >= 200 && http_code < 300) {
                    RCLCPP_DEBUG(this->get_logger(), "Interaction for step %d logged successfully", step_id);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Failed to log interaction for step %d (HTTP %ld): %s",
                        step_id,
                        http_code,
                        body.c_str());
                }
            });

        RCLCPP_DEBUG(this->get_logger(), "Interaction logged for step %d: response='%s', correct=%s",
            step_id, response.c_str(), is_correct ? "true" : "false");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error logging interaction: %s", e.what());
    }
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


