#ifndef BLOOM_NODE_LESSON_COORDINATOR_H
#define BLOOM_NODE_LESSON_COORDINATOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "bloom_msgs/action/play_behavior.hpp"
#include "bloom_node/behavior_coordinator.h"
#include "bloom_node/web_service_client.h"
#include "bloom_node/state_manager.h"
#include "bloom_node/feedback_poller.h"
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <optional>
#include <atomic>
#include <std_msgs/msg/string.hpp>

namespace bloom_node {
struct InteractionConfig {
    bool wait_for_response;
    int max_wait_seconds;
    std::string correct_answer;
    std::string correct_response_script;
    std::string incorrect_response_script;
    std::string fallback_script;
    bool llm_follow_up{false};
    bool single_turn_llm{false};          
    std::string single_turn_llm_prompt;   
    std::vector<std::string> fallback_visual_aid;
    std::vector<std::string> fallback_visual_aid_labels;
};

struct LessonStep {
    std::string id;        
    int step_order;        
    std::string type;
    std::string script;
    std::map<std::string, std::string> behaviors;  // gesture, facial_expression, gaze, etc.
    int timing_seconds;
    std::string visual_aid_url;
    bool has_interaction;
    InteractionConfig interaction;
    std::vector<std::string> visual_aid_images;
    std::vector<std::string> visual_aid_labels;
    std::vector<std::string> visual_aid_footers;
    std::string motor_sequence;
    
};

struct LessonData {
    std::string lesson_id;
    std::string lesson_run_id;
    std::string title;
    std::vector<std::string> learning_objectives;
    std::vector<LessonStep> sequence;
    bool conversation_mode{false};

};

/**
 * Lesson execution tracking and coordination for educational modules.
 */

class LessonCoordinator : public rclcpp::Node {

public:
    explicit LessonCoordinator(
        std::shared_ptr<BehaviorCoordinator> behavior_coordinator,
        std::shared_ptr<bloom_node::WebServiceClient> web_client,
        std::shared_ptr<StateManager> state_manager,
        const std::string &node_name = "lesson_coordinator_node"
    );

    ~LessonCoordinator();

    // load lesson data from the backend and prepare for execution
    bool load_lesson(const LessonData &lesson_data);

    // start executing the loaded lesson from the beginning
    void start_lesson();

    // stop the lesson execution immediately and call idle state
    void stop_lesson();

    // reset the lesson progress to the beginning
    void reset_lesson();

    // Check if a lesson is currently executing
    bool is_lesson_running() const;

    // Set a callback to be invoked when the lesson completes
    using LessonCompletionCallback = std::function<void(const std::string &lesson_id)>;
    void set_completion_callback(LessonCompletionCallback callback);

    // Set the feedback poller to control during interactions
    void set_feedback_poller(std::shared_ptr<FeedbackPoller> feedback_poller);

    // Set the session ID (called when user joins with pairing code and userId is set)
    void set_session_id(const std::string &session_id);

    // Get the current session ID
    std::string get_session_id() const;

    // Step control, called by LessonPoller when SLP issues commands
    void skip_step();
    void replay_step();
    void set_step(int target_step_order);

    // Pause execution in place (interrupts immediately, keeps current_step_index_
    // intact) and resume by re-playing the current step from its beginning.
    // Called by LessonPoller when SLP issues pause/resume commands.
    void pause_lesson();
    void resume_lesson();

    // Idle-mode control (no lesson active). "conversational" turns on free-form
    // STT/LLM/TTS chat; "passive" (default) is plain breathing/idle with mic off.
    // No-ops while a lesson is active. Idempotent — no-ops if mode is already
    // applied, so LessonPoller can call this every poll tick without restarting
    // StateManager's behavior loop.
    void set_idle_mode(const std::string &mode);

    // Sets the robot's TTS voice for subsequent speech. Idempotent — no-ops if
    // the voice hasn't changed. Not gated by lesson state; applies any time.
    void set_tts_voice(const std::string &voice);

private:

    void execute_step(const LessonStep &step);
    void queue_behavior(const LessonStep &step);
    void speak_script(const std::string &script);

    void handle_interaction(const LessonStep &step);
    void on_vosk_result(const std_msgs::msg::String::SharedPtr msg);

    void schedule_next_step(int delay_seconds);
    void advance_to_next_step();

    // Visual aid download pipeline: resolves each entry to a local cache filename
    // (bare filenames pass through unchanged for pre-bundled content; "http(s)://"
    // URLs and backend-relative paths are downloaded via web_client_ into the same
    // share directory bloom_face already resolves local filenames against), then
    // publishes once every image in the step is ready (downloaded or failed).
    std::string resolve_visual_aid_cache_dir();
    std::string visual_aid_cache_filename(const std::string &entry) const;
    void publish_visual_aid_message(
        const std::vector<std::string> &filenames,
        const std::vector<std::string> &labels,
        const std::vector<std::string> &footers);
    void resolve_and_publish_visual_aids(const LessonStep &step, uint64_t generation);

    void update_progress_with_backend();
    void log_interaction_to_backend(int step_order, const std::string &response, bool is_correct);
    void log_interaction_to_backend(int step_order, const std::string &interaction_type, const std::string &content, std::optional<bool> is_correct);

    void on_tts_done(const std_msgs::msg::String::SharedPtr msg);
    void on_llm_wrap_up(const std_msgs::msg::String::SharedPtr msg);

    LessonData current_lesson_;
    size_t current_step_index_;
    bool lesson_active_;
    std::string lesson_progress_id_;
    std::string session_id_;
    std::string lesson_run_id_;

    // For interaction handling
    LessonStep* current_interaction_step_;
    bool waiting_for_response_;

    // True while paused: lesson_active_ stays true, current_step_index_ is
    // preserved, execution is frozen until resume_lesson() re-plays the step.
    bool lesson_paused_{false};

    // Visual aid download pipeline state. Bumped once per execute_step() call so a
    // slow/in-flight download's completion can detect it's been superseded by a
    // later step (skip/replay/set_step/pause/stop) and discard its result instead
    // of publishing a stale image.
    std::atomic<uint64_t> visual_aid_generation_{0};
    std::string visual_aids_cache_dir_;

    std::shared_ptr<BehaviorCoordinator> behavior_coordinator_;
    std::shared_ptr<bloom_node::WebServiceClient> web_client_;
    std::shared_ptr<StateManager> state_manager_;
    std::shared_ptr<FeedbackPoller> feedback_poller_;


    std::mutex lesson_mutex_;

    // Completion callback
    LessonCompletionCallback completion_callback_;

    rclcpp::TimerBase::SharedPtr step_timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lesson_progress_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_voice_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_aid_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vosk_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr llm_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr llm_context_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_done_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wrap_up_sub_;
    rclcpp_action::Client<bloom_msgs::action::PlayBehavior>::SharedPtr motor_action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stt_enable_pub_;
    bool waiting_for_tts_done_{false};
    bool waiting_for_wrap_up_{false};
    bool waiting_for_interaction_tts_{false};
    bool waiting_for_single_turn_{false};
    bool waiting_for_llm_tts_done_{false};
    bool conversation_mode_{false};

    // Current idle mode ("passive" or "conversational"), tracked so
    // set_idle_mode() can no-op on repeated identical polls.
    std::string current_idle_mode_{"passive"};

    // Current TTS voice, tracked so set_tts_voice() can no-op on repeated
    // identical polls. Empty until the first voice is received.
    std::string current_tts_voice_{};

    std::string robot_state_{"idle"};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_interrupt_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chime_pub_;

};
}

#endif //BLOOM_NODE_LESSON_COORDINATOR_H