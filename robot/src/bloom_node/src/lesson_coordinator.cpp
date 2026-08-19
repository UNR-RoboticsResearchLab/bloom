#include "bloom_node/lessson_coordinator.h"
#include <sstream>
#include <mutex>
#include <functional>
#include <regex>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
    motor_action_client_ = rclcpp_action::create_client<bloom_msgs::action::PlayBehavior>(
        this, "play_behavior");
    stt_enable_pub_ = this->create_publisher<std_msgs::msg::String>("/stt/enable", 10);
    robot_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot/state", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            robot_state_ = msg->data;
        });
    RCLCPP_INFO(this->get_logger(), "LessonCoordinator initialized with Vosk subscriber");

    tts_interrupt_pub_ = this->create_publisher<std_msgs::msg::String>("/tts/interrupt", 10);
    chime_pub_ = this->create_publisher<std_msgs::msg::String>("/audio/chime", 10);
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
    conversation_mode_ = lesson_data.conversation_mode;
    current_lesson_ = lesson_data;
    lesson_run_id_ = lesson_data.lesson_run_id;
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
    lesson_paused_ = false;
    current_step_index_ = 0;

    RCLCPP_INFO(this->get_logger(), "[LESSON_START] Starting lesson: %s with %zu steps", current_lesson_.title.c_str(), current_lesson_.sequence.size());
    RCLCPP_INFO(this->get_logger(), "[LESSON_STATE] lesson_active_=%s, calling advance_to_next_step()", lesson_active_ ? "true" : "false");
    advance_to_next_step();
}

void LessonCoordinator::stop_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);

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
    conversation_mode_ = false;

    auto stt_msg = std_msgs::msg::String();
    stt_msg.data = "false";
    stt_enable_pub_->publish(stt_msg);

    auto interrupt_msg = std_msgs::msg::String();
    interrupt_msg.data = "interrupt";
    tts_interrupt_pub_->publish(interrupt_msg);

    auto va_msg = std_msgs::msg::String();
    va_msg.data = "{\"command\": \"hide\"}";
    visual_aid_publisher_->publish(va_msg);

    auto mode_msg = std_msgs::msg::String();
    mode_msg.data = "lesson_mode";
    llm_mode_pub_->publish(mode_msg);

    if (state_manager_) state_manager_->set_state("idle");
    if (feedback_poller_) feedback_poller_->set_polling_active(false);

    lesson_active_ = false;
    lesson_paused_ = false;

    if (completion_callback_) {
        completion_callback_(current_lesson_.lesson_id);
    }

    RCLCPP_INFO(this->get_logger(), "[STOP] Lesson stopped by SLP command");
}

void LessonCoordinator::reset_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);

    if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = nullptr;
    }

    lesson_active_ = false;
    lesson_paused_ = false;
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

    if (!step.motor_sequence.empty() && motor_action_client_) {
        if (!motor_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(this->get_logger(),
                "play_behavior action server not available - dropping motor sequence: %s",
                step.motor_sequence.c_str());
        } else {
            bloom_msgs::action::PlayBehavior::Goal goal;
            goal.behavior_name = step.motor_sequence;

            rclcpp_action::Client<bloom_msgs::action::PlayBehavior>::SendGoalOptions options;
            auto logger = this->get_logger();
            std::string sequence_name = step.motor_sequence;
            options.goal_response_callback =
                [logger, sequence_name](
                    const rclcpp_action::ClientGoalHandle<bloom_msgs::action::PlayBehavior>::SharedPtr & goal_handle) {
                    if (!goal_handle) {
                        RCLCPP_WARN(logger, "Motor sequence rejected by play_behavior action server: %s",
                            sequence_name.c_str());
                    }
                };
            options.result_callback =
                [logger, sequence_name](
                    const rclcpp_action::ClientGoalHandle<bloom_msgs::action::PlayBehavior>::WrappedResult & result) {
                    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                        RCLCPP_WARN(logger, "Motor sequence did not complete successfully: %s",
                            sequence_name.c_str());
                    }
                };

            motor_action_client_->async_send_goal(goal, options);
            RCLCPP_INFO(this->get_logger(), "Playing motor sequence: %s", step.motor_sequence.c_str());
        }
    }

    // Publish visual aid or hide
    if (!step.visual_aid_images.empty()) {
        uint64_t generation = ++visual_aid_generation_;
        resolve_and_publish_visual_aids(step, generation);
    } else {
        auto va_msg = std_msgs::msg::String();
        va_msg.data = "{\"command\": \"hide\"}";
        visual_aid_publisher_->publish(va_msg);
    }

    if (step.interaction.llm_follow_up) {
        auto ctx_msg = std_msgs::msg::String();
        ctx_msg.data = "Lesson topic: " + current_lesson_.title + ". Current question: " + step.script;
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

std::string LessonCoordinator::resolve_visual_aid_cache_dir() {
    if (visual_aids_cache_dir_.empty()) {
        try {
            visual_aids_cache_dir_ = ament_index_cpp::get_package_share_directory("bloom_face") + "/visual_aids";
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                "Could not resolve bloom_face visual_aids share dir (%s); falling back to /tmp", e.what());
            visual_aids_cache_dir_ = "/tmp/bloom_visual_aids";
        }
        std::error_code ec;
        std::filesystem::create_directories(visual_aids_cache_dir_, ec);
        if (ec) {
            RCLCPP_WARN(this->get_logger(), "Could not create visual aid cache dir %s: %s",
                visual_aids_cache_dir_.c_str(), ec.message().c_str());
        }
    }
    return visual_aids_cache_dir_;
}

std::string LessonCoordinator::visual_aid_cache_filename(const std::string &entry) const {
    bool is_remote = entry.rfind("http://", 0) == 0 ||
                      entry.rfind("https://", 0) == 0 ||
                      entry.rfind("/", 0) == 0;
    if (!is_remote) {
        // Bare filename referring to pre-bundled content in visual_aids/ — unchanged
        // legacy behavior, resolved by bloom_face exactly as it is today.
        return entry;
    }

    // Deterministic cache filename so repeat views are a disk hit, not a re-download.
    std::string ext;
    auto dot = entry.find_last_of('.');
    auto slash = entry.find_last_of('/');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash) && entry.size() - dot <= 8) {
        ext = entry.substr(dot);
    }
    size_t hash = std::hash<std::string>{}(entry);
    std::ostringstream oss;
    oss << std::hex << hash << ext;
    return oss.str();
}

void LessonCoordinator::publish_visual_aid_message(
    const std::vector<std::string> &filenames,
    const std::vector<std::string> &labels,
    const std::vector<std::string> &footers)
{
    nlohmann::json va_json;
    va_json["images"] = filenames;
    va_json["labels"] = labels;
    if (!footers.empty()) {
        va_json["footers"] = footers;
    }
    auto va_msg = std_msgs::msg::String();
    va_msg.data = va_json.dump();
    visual_aid_publisher_->publish(va_msg);
}

void LessonCoordinator::resolve_and_publish_visual_aids(const LessonStep &step, uint64_t generation) {
    std::string cache_dir = resolve_visual_aid_cache_dir();

    auto filenames = std::make_shared<std::vector<std::string>>();
    filenames->reserve(step.visual_aid_images.size());
    for (const auto &entry : step.visual_aid_images) {
        filenames->push_back(visual_aid_cache_filename(entry));
    }

    // index into step.visual_aid_images / *filenames, raw source (URL or relative path)
    std::vector<std::pair<size_t, std::string>> to_download;
    for (size_t i = 0; i < step.visual_aid_images.size(); ++i) {
        std::string full_path = cache_dir + "/" + (*filenames)[i];
        std::error_code ec;
        if (!std::filesystem::exists(full_path, ec)) {
            to_download.emplace_back(i, step.visual_aid_images[i]);
        }
    }

    auto labels = step.visual_aid_labels;
    auto footers = step.visual_aid_footers;

    if (to_download.empty()) {
        // Everything already cached (or was pre-bundled all along) — publish
        // immediately, exactly matching pre-download-pipeline behavior/latency.
        publish_visual_aid_message(*filenames, labels, footers);
        return;
    }

    if (!web_client_) {
        RCLCPP_WARN(this->get_logger(),
            "No web client available to download %zu visual aid(s); publishing without them",
            to_download.size());
        publish_visual_aid_message(*filenames, labels, footers);
        return;
    }

    auto pending = std::make_shared<std::atomic<size_t>>(to_download.size());

    for (const auto &pair : to_download) {
        std::string dest = cache_dir + "/" + (*filenames)[pair.first];
        web_client_->downloadFileAsync(pair.second, dest,
            [this, generation, pending, filenames, labels, footers]
            (bool success, const std::string &filePath, const std::string &error) {
                if (!success) {
                    RCLCPP_WARN(this->get_logger(), "Visual aid download failed for %s: %s",
                        filePath.c_str(), error.c_str());
                }
                if (pending->fetch_sub(1) == 1) {
                    // Last download in this step's batch has resolved (success or not).
                    std::lock_guard<std::mutex> lock(lesson_mutex_);
                    if (generation != visual_aid_generation_.load()) {
                        return; // superseded by a later step/command — discard
                    }
                    publish_visual_aid_message(*filenames, labels, footers);
                }
            });
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
            mode_msg.data = conversation_mode_ ? "conversation_lesson" : "lesson_tangent";
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
                auto chime_msg = std_msgs::msg::String();
                chime_msg.data = "play";
                chime_pub_->publish(chime_msg);
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
                            mode_msg.data = conversation_mode_ ? "conversation_lesson" : "lesson_mode";
                            llm_mode_pub_->publish(mode_msg);

                            if (!step->interaction.fallback_visual_aid.empty()) {
                                nlohmann::json va_json;
                                va_json["images"] = step->interaction.fallback_visual_aid;
                                va_json["labels"] = step->interaction.fallback_visual_aid_labels;
                                auto va_msg = std_msgs::msg::String();
                                va_msg.data = va_json.dump();
                                visual_aid_publisher_->publish(va_msg);
                            }

                            log_interaction_to_backend(step->step_order, "timeout", false);

                            if (!step->interaction.fallback_script.empty()) {
                                speak_script(step->interaction.fallback_script);
                                waiting_for_tts_done_ = true;
                            } else {
                                advance_to_next_step();
                            }
                        });
                }
            });
        return;
    }

    // LLM spoke a tangent response without [RETURN_TO_LESSON] — re-open mic for next exchange
    if (waiting_for_wrap_up_ && !waiting_for_tts_done_ && !waiting_for_interaction_tts_ && !waiting_for_llm_tts_done_) {
        RCLCPP_INFO(this->get_logger(), "[TTS_DONE] LLM tangent response done — re-opening mic");
        step_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                step_timer_->cancel();
                step_timer_ = nullptr;
                auto stt_msg = std_msgs::msg::String();
                stt_msg.data = "true";
                auto chime_msg = std_msgs::msg::String();
                chime_msg.data = "play";
                chime_pub_->publish(chime_msg);
                stt_enable_pub_->publish(stt_msg);
                waiting_for_response_ = true;
                RCLCPP_INFO(this->get_logger(), "Now listening for student response (tangent continue)");
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
        mode_msg.data = conversation_mode_ ? "conversation_lesson" : "lesson_mode";
        llm_mode_pub_->publish(mode_msg);
        waiting_for_llm_tts_done_ = true;
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

    int step_order = 0;
    if (current_interaction_step_) {
        step_order = current_interaction_step_->step_order;
    } else if (current_step_index_ > 0 && current_step_index_ <= current_lesson_.sequence.size()) {
        step_order = current_lesson_.sequence[current_step_index_ - 1].step_order;
    }
    log_interaction_to_backend(step_order, "robot", script, std::nullopt);
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

        if (interaction.single_turn_llm) {
            RCLCPP_INFO(this->get_logger(), "[VOSK] single_turn_llm step — passing to LLM");
            
            log_interaction_to_backend(step.step_order, "student", response, std::nullopt);
            
            waiting_for_response_ = false;
            
            auto stt_msg = std_msgs::msg::String();
            stt_msg.data = "false";
            stt_enable_pub_->publish(stt_msg);
            
            if (feedback_poller_) feedback_poller_->set_polling_active(false);
            
            if (step_timer_) {
                step_timer_->cancel();
                step_timer_ = nullptr;
            }
            
            waiting_for_single_turn_ = true;
            waiting_for_llm_tts_done_ = false;
            return;
        }
        if (interaction.llm_follow_up) {
            RCLCPP_INFO(this->get_logger(), "[VOSK] llm_follow_up step — waiting for LLM wrap-up");
            
            log_interaction_to_backend(step.step_order, "student", response, std::nullopt);
            
            waiting_for_response_ = false;
            
            auto stt_msg = std_msgs::msg::String();
            stt_msg.data = "false";
            stt_enable_pub_->publish(stt_msg);
            
            if (feedback_poller_) feedback_poller_->set_polling_active(false);
            
            if (step_timer_) {
                step_timer_->cancel();
                step_timer_ = nullptr;
            }
            return;  
        }   

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

        // Only wait for TTS if we actually spoke something
        bool spoke_feedback = (is_correct && !interaction.correct_response_script.empty()) ||
                            (!is_correct && !interaction.incorrect_response_script.empty());

        if (spoke_feedback) {
            waiting_for_tts_done_ = true;
        } else {
            advance_to_next_step();
        }

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
    if (current_step.type == "loop") {
        RCLCPP_INFO(this->get_logger(), "[LOOP] Restarting demo lesson");
        current_step_index_ = 0;
        update_progress_with_backend();
        execute_step(current_lesson_.sequence[0]);
        return;
    }

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
    const std::string type = (response == "timeout") ? "Timeout" : "student";
    log_interaction_to_backend(step_order, type, response, std::optional<bool>(is_correct));
}

void LessonCoordinator::log_interaction_to_backend(int step_order, const std::string &interaction_type, const std::string &content, std::optional<bool> is_correct) {
    try {
        if (!web_client_ || session_id_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot log interaction: web_client or session_id missing");
            return;
        }

        json payload;
        payload["stepId"] = step_order;
        payload["interactionType"] = interaction_type;
        payload["studentResponse"] = content;
        if (is_correct.has_value()) {
            payload["isCorrect"] = is_correct.value();
        } else {
            payload["isCorrect"] = nullptr;
        }
        payload["responseTimeMs"] = 0;
        if (!lesson_run_id_.empty()) {
            payload["lessonRunId"] = lesson_run_id_;
        }

        std::string interaction_endpoint = "/api/lessoninteraction/" + session_id_;

        web_client_->sendRequestAsync(
            "POST",
            interaction_endpoint,
            payload.dump(),
            std::nullopt,
            {"Content-Type: application/json"},
            [this, step_order, interaction_type](const std::string &body, long http_code) {
                if (http_code >= 200 && http_code < 300) {
                    RCLCPP_DEBUG(this->get_logger(), "%s for step %d logged successfully", interaction_type.c_str(), step_order);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Failed to log %s for step %d (HTTP %ld): %s",
                        interaction_type.c_str(),
                        step_order,
                        http_code,
                        body.c_str());
                }
            });

        RCLCPP_DEBUG(this->get_logger(), "%s logged for step %d: '%s'",
            interaction_type.c_str(), step_order, content.c_str());
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
    if (lesson_paused_) {
        RCLCPP_WARN(this->get_logger(), "skip_step called while paused - ignoring");
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
    if (lesson_paused_) {
        RCLCPP_WARN(this->get_logger(), "replay_step called while paused - ignoring");
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
    if (lesson_paused_) {
        RCLCPP_WARN(this->get_logger(), "set_step called while paused - ignoring");
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

void LessonCoordinator::pause_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    if (!lesson_active_) {
        RCLCPP_WARN(this->get_logger(), "pause_lesson called but no lesson active");
        return;
    }
    if (lesson_paused_) {
        RCLCPP_WARN(this->get_logger(), "pause_lesson called but lesson already paused");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[PAUSE] Pausing lesson at step index %zu", current_step_index_);

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

    // Deliberately leave the visual aid on screen and skip the completion
    // callback: pausing must not stop LessonPoller's step-control polling,
    // or a subsequent resume command would never be received.
    if (state_manager_) state_manager_->set_state("waiting");
    if (feedback_poller_) feedback_poller_->set_polling_active(false);

    lesson_paused_ = true;

    RCLCPP_INFO(this->get_logger(), "[PAUSE] Lesson paused by SLP command");
}

void LessonCoordinator::resume_lesson() {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    if (!lesson_active_ || !lesson_paused_) {
        RCLCPP_WARN(this->get_logger(), "resume_lesson called but lesson not paused");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[RESUME] Resuming lesson at step index %zu",
        current_step_index_ > 0 ? current_step_index_ - 1 : 0);

    lesson_paused_ = false;

    if (state_manager_) state_manager_->set_state("waiting");
    if (feedback_poller_) feedback_poller_->set_polling_active(false);

    // Re-play the current step from its beginning — same index math as
    // replay_step(): step back one so advance_to_next_step() re-executes it.
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

    RCLCPP_INFO(this->get_logger(), "[RESUME] Lesson resumed by SLP command");
}

void LessonCoordinator::set_idle_mode(const std::string &mode) {
    std::lock_guard<std::mutex> lock(lesson_mutex_);
    if (lesson_active_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring set_idle_mode(%s) — lesson is active", mode.c_str());
        return;
    }
    if (mode == current_idle_mode_) return;  // avoid restarting behavior loop every tick
    current_idle_mode_ = mode;

    auto mode_msg = std_msgs::msg::String();
    auto stt_msg = std_msgs::msg::String();

    if (mode == "conversational") {
        mode_msg.data = "free_conversation";
        stt_msg.data = "true";
    } else {
        // "passive" — reuse "lesson_mode" as the neutral LLM mode value: stt_node.py
        // already treats it as mic-off, and llm_node.py already resets to its
        // no-lesson-context prompt for it.
        mode_msg.data = "lesson_mode";
        stt_msg.data = "false";
    }
    llm_mode_pub_->publish(mode_msg);
    stt_enable_pub_->publish(stt_msg);

    // Both idle modes reuse the existing "waiting" state (idle/breathing behavior
    // loop + neutral face) — tts_node.py hardcodes a revert to "waiting" after
    // every spoken utterance, so a distinct state here would just get clobbered.
    if (state_manager_) state_manager_->set_state("waiting");

    RCLCPP_INFO(this->get_logger(), "[IDLE] Idle mode set to %s", mode.c_str());
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


