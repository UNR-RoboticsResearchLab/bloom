#include "bloom_node/lessson_coordinator.h"

using namespace bloom_node;

LessonCoordinator::LessonCoordinator(
    std::shared_ptr<BehaviorCoordinator> behavior_coordinator,
    std::shared_ptr<web_service_client> web_client,
    const std::string &node_name
) : Node(node_name),
    behavior_coordinator_(behavior_coordinator),
    web_client_(web_client),
    current_step_index_(0),
    lesson_active_(false) {
    lesson_progress_publisher_ = this->create_publisher<std_msgs::msg::String>("lesson_progress", 10);
    tts_publisher_ = this->create_publisher<std_msgs::msg::String>("tts_output", 10);
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


    queue_behavior(step);
    speak_script(step.script);

    if (step.has_interaction) {
        handle_interaction(step);
    }
}

void LessonCoordinator::queue_behavior(const LessonStep &step) {
    // Queue behaviors to the behavior coordinator
    for (const auto &[behavior_type, behavior_value] : step.behaviors) {
        RCLCPP_DEBUG(this->get_logger(), "Queueing behavior: %s = %s", behavior_type.c_str(), behavior_value.c_str());
    
        behavior_coordinator_->queue_behavior(behavior_type, behavior_value);
    }
}

void LessonCoordinator::speak_script(const std::string &script) {
    auto message = std_msgs::msg::String();
    message.data = script;
    tts_publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Speaking: %s", script.c_str());
}

void LessonCoordinator::handle_interaction(const LessonStep &step) {
    RCLCPP_INFO(this->get_logger(), "Handling interaction for step %d", step.id);
    // TODO: Implement interaction handling
}

void LessonCoordinator::advance_to_next_step() {
    if (!lesson_active_) {
        return;
    }

    if (current_step_index_ >= current_lesson_.sequence.size()) {
        RCLCPP_INFO(this->get_logger(), "Lesson completed");
        lesson_active_ = false;
        return;
    }

    const LessonStep &current_step = current_lesson_.sequence[current_step_index_];
    execute_step(current_step);

    current_step_index_++;
}

void LessonCoordinator::update_progress_with_backend() {
    // TODO: Send lesson progress to backend
}

void LessonCoordinator::log_interaction_to_backend(const std::string &interaction_result) {
    // TODO: Log interaction result to backend
}
