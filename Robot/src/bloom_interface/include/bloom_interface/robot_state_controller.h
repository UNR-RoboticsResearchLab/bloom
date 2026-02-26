// bloom
// robot_state_controller.h
// Header file for RobotStateController class managing high level robot state and communication with backend.
// Created: 12/2/2025

#ifndef ROBOT_STATE_CONTROLLER_H
#define ROBOT_STATE_CONTROLLER_H

#include <memory>
#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <bloom_node/web_service_client.h>
#include <bloom_node/configuration_manager.h>

namespace robot_state_controller {


class RobotStateController : public rclcpp::Node {
public:
    using Ptr = std::shared_ptr<RobotStateController>;

    /// Constructor
    /// @param web_client Shared WebServiceClient for HTTP communication
    /// @param config_mgr Shared ConfigurationManager for configuration access
    explicit RobotStateController(
        std::shared_ptr<bloom_node::WebServiceClient> web_client,
        std::shared_ptr<bloom_node::ConfigurationManager> config_mgr
    );

    ~RobotStateController() override = default;

    // Non-copyable
    RobotStateController(const RobotStateController &) = delete;
    RobotStateController & operator=(const RobotStateController &) = delete;

    /// Get current robot state (thread-safe read)
    std::string get_state() const;

    /// Get current task (thread-safe read)
    std::string get_task() const;

    /// Get robot ID (thread-safe read)
    std::string get_robot_id() const;

    /// Get session ID (thread-safe read)
    std::string get_session_id() const;

private:
    // ========== State Management ==========
    std::string current_state_;      // Robot state (waiting, speaking, loading, etc)
    std::string current_task_;       // Current task (giving lesson, free speech, etc)
    std::string robot_id_;           // UUID of registered robot
    std::string session_id_;         // UUID of active session
    mutable std::mutex state_mutex_; // Mutex for thread-safe state access

    // ========== ROS2 Components ==========
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr driver_state_sub_;
    rclcpp::TimerBase::SharedPtr state_sync_timer_;

    // ========== Backend Communication ==========
    std::shared_ptr<bloom_node::WebServiceClient> web_client_;
    std::shared_ptr<bloom_node::ConfigurationManager> config_mgr_;

    // ========== Initialization ==========
    /// Register robot with C# backend on startup
    void register_robot();

    /// Start anonymous session on C# backend
    void start_session();

    /// Add registered robot to session
    void add_robot_to_session();

    // ========== State Synchronization ==========
    /// Callback for driver state updates
    /// @param msg State message from driver
    void driver_state_callback(const std_msgs::msg::String::SharedPtr msg);

    /// Sync current state to C# backend (timer callback)
    void sync_state_with_backend();

    /// Send HTTP request to backend
    /// @param method HTTP method (GET, POST, PUT, DELETE)
    /// @param endpoint API endpoint path
    /// @param payload JSON payload (if needed)
    void send_backend_request(
        const std::string& method,
        const std::string& endpoint,
        const std::string& payload = ""
    );

    // ========== Command Handling ==========
    /// Handle commands from C# backend (stub for future implementation)
    /// @param command Command string from backend
    void backend_command_callback(const std::string& command);

    // ========== Helper Methods ==========
    /// Get current timestamp in ISO 8601 format
    std::string get_current_timestamp() const;

    /// Parse JSON response from backend
    /// @param response JSON response string
    /// @param key Key to extract from response
    /// @return Value of key, or empty string if not found
    std::string extract_json_value(const std::string& response, const std::string& key) const;
};

} // namespace robot_state_controller

#endif // ROBOT_STATE_CONTROLLER_H
