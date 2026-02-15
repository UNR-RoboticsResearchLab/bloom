#include "bloom_node/lesson_coordinator.h"
#include "bloom_node/lessson_coordinator.h"

using namespace bloom_node;

LessonCoordinator::LessonCoordinator(
    std::shared_ptr<BehaviorCoordinator> behavior_coordinator,
    std::shared_ptr<web_service_client> web_client,
    const std::string &node_name
) : Node(node_name), behavior_coordinator_(behavior_coordinator), web_client_(web_client) {
    lesson_progress_publisher_ = this->create_publisher<std_msgs::msg::String>("lesson_progress", 10);
    tts_publisher_ = this->create_publisher<std_msgs::msg::String>("tts_output", 10);
}

LessonCoordinator::~LessonCoordinator() {
    stop_lesson();
}

bool LessonCoordinator::load_lesson(const LessonData &lesson_data) {
    

    web_client_->sendGetAsync("/api/lessons/" + lesson_data.lesson_id, {}, [this](const std::string &body, long http_code) {
        if (http_code == 200) {
            RCLCPP_INFO(this->get_logger(), "Lesson data loaded successfully from backend");
            //parse json into lesson
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load lesson data from backend: HTTP %ld", http_code);
        }
    });



}