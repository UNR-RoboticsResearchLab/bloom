#ifndef BLOOM_NODE_LESSON_COORDINATOR_H
#define BLOOM_NODE_LESSON_COORDINATOR_H

#include <rclcpp/rclcpp.hpp>
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

private:

    void execute_step(const LessonStep &step);
    void queue_behavior(const LessonStep &step);
    void speak_script(const std::string &script);

    void handle_interaction(const LessonStep &step);
    void on_vosk_result(const std_msgs::msg::String::SharedPtr msg);

    void schedule_next_step(int delay_seconds);
    void advance_to_next_step();

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_aid_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vosk_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr llm_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr llm_context_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_done_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wrap_up_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stt_enable_pub_;
    bool waiting_for_tts_done_{false};
    bool waiting_for_wrap_up_{false};
    bool waiting_for_interaction_tts_{false};
    bool waiting_for_single_turn_{false};
    bool waiting_for_llm_tts_done_{false};
    bool conversation_mode_{false};

    std::string robot_state_{"idle"};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_interrupt_pub_;


};
}

#endif //BLOOM_NODE_LESSON_COORDINATOR_H