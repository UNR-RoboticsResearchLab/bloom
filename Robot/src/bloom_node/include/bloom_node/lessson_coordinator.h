#ifndef BLOOM_NODE_LESSON_COORDINATOR_H
#define BLOOM_NODE_LESSON_COORDINATOR_H

#include "bloom_node/behavior_coordinator.h"
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <mutex>
#include <chrono>
#include <algorithm>

namespace bloom_node {

/**
 * Lesson execution tracking and coordination for educational modules.
 */

    struct LessonStep {
    int id;
    std::string type;
    std::string script;
    std::map<std::string, std::string> behaviors;  // gesture, facial_expression, gaze, etc.
    int timing_seconds;
    bool has_interaction;
    InteractionConfig interaction;
};

struct InteractionConfig {
    bool wait_for_response;
    int max_wait_seconds;
    std::string correct_answer;
    std::string correct_response_script;
    std::string incorrect_response_script;
    std::string fallback_script;
};

struct LessonData {
    std::string lesson_id;
    std::string title;
    std::vector<LessonStep> sequence;
};

class LessonCoordinator : public rclcpp::Node {

public:
    explicit LessonCoordinator(
        std::shared_ptr<BehaviorCoordinator> behavior_coordinator,
        std::shared_ptr<web_service_client> web_client,
        const std::string &node_name = "lesson_coordinator_node"
    );

    bool load_lesson(const LessonData &lesson_data);

    void start_lesson();

    void stop_lesson();

    void reset_lesson();

private:

    void execute_step(const LessonStep &step);
    void queue_behavior(const LessonStep &step);
    void speak_script(const std::string &script);

    void handle_interaction(const LessonStep &step);

    void advance_to_next_step();

    void update_progress_with_backend();
    void log_interaction_to_backend(const std::string &interaction_result);


    LessonData current_lesson_;
    size_t current_step_index_;
    bool lesson_active_;
    std::string lesson_progress_id_;
    std::string session_id_;

    std::shared_ptr<BehaviorCoordinator> behavior_coordinator_;
    std::shared_ptr<web_service_client> web_client_;
    std::mutex lesson_mutex_;

    rclcpp::TimerBase::SharedPtr step_timer_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lesson_progress_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_publisher_;


};