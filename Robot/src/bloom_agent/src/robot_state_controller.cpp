#include "bloom_agent/robot_state_controller.h"
#include <bloom_node/json.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <uuid/uuid.h>

using json = nlohmann::json;

namespace robot_state_controller {

RobotStateController::RobotStateController(
    std::shared_ptr<web_service_client::WebServiceClient> web_client,
    std::shared_ptr<configuration_manager::ConfigurationManager> config_mgr)
    : rclcpp::Node("robot_state_controller"),
      current_state_("waiting"),
      current_task_("idle"),
      robot_id_(""),
      session_id_(""),
      web_client_(web_client),
      config_mgr_(config_mgr) {

    RCLCPP_INFO(get_logger(), "Initializing RobotStateController...");

    try {
        // TODO: check config for registration/jwt tokens - unguarded registration for now
        // Register robot
        RCLCPP_INFO(get_logger(), "Registering robot with backend...");
        register_robot();

        // Start session
        RCLCPP_INFO(get_logger(), "Starting anonymous session...");
        start_session();

        // Add robot to session
        RCLCPP_INFO(get_logger(), "Adding robot to session...");
        add_robot_to_session();

        // Subscribe to driver state updates
        RCLCPP_INFO(get_logger(), "Subscribing to driver state topic...");
        //TODO: Actually do this when driver is ready

        // Start periodic state sync timer (5 seconds)
        RCLCPP_INFO(get_logger(), "Starting state sync timer (5s interval)...");
        state_sync_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RobotStateController::sync_state_to_backend, this)
        );

        RCLCPP_INFO(get_logger(), "RobotStateController initialized successfully!");
        RCLCPP_INFO(get_logger(), "Robot ID: %s", robot_id_.c_str());
        RCLCPP_INFO(get_logger(), "Session ID: %s", session_id_.c_str());

    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize RobotStateController: %s", e.what());
        throw;
    }
}

void RobotStateController::register_robot() {
    // Get robot configuration
    std::string robot_name = config_mgr_->get_string("robot_name").value_or("agetha");
    std::string robot_serial = config_mgr_->get_string("robot_serial").value_or("BLSM-001");
    std::string robot_ip = config_mgr_->get_string("robot_ip").value_or("192.168.1.108");
    std::string firmware_version = config_mgr_->get_string("firmware_version").value_or("1.0.0");

    // Create registration payload
    json payload = {
        {"name", robot_name},
        {"model", "Blossom"},
        {"serialNumber", robot_serial},
        {"manufactureDate", "2025-12-01"},
        {"firmwareVersion", firmware_version},
        {"ipAddress", robot_ip}
    };

    // Send registration request (blocking - using promise for synchronous behavior)
    bool registration_done = false;
    std::string registration_error;

    web_client_->sendJsonPostAsync(
        "/api/robots/register",
        payload,
        {"Content-Type: application/json"},
        [this, &registration_done, &registration_error](const std::string& body, long http_code) {
            if (http_code >= 200 && http_code < 300) {
                try {
                    auto response = json::parse(body);
                    if (response.contains("id")) {
                        robot_id_ = response["id"].get<std::string>();
                        RCLCPP_INFO(get_logger(), "Robot registered successfully. ID: %s", robot_id_.c_str());
                    } else {
                        registration_error = "No ID in registration response";
                    }
                } catch (const std::exception& e) {
                    registration_error = std::string("Failed to parse registration response: ") + e.what();
                }
            } else {
                registration_error = "HTTP " + std::to_string(http_code) + ": " + body;
            }
            registration_done = true;
        }
    );

    // Wait for registration to complete (max 10 seconds)
    auto start_time = std::chrono::steady_clock::now();
    while (!registration_done && std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!registration_done) {
        throw std::runtime_error("Robot registration timeout");
    }
    if (!registration_error.empty()) {
        throw std::runtime_error("Robot registration failed: " + registration_error);
    }
    if (robot_id_.empty()) {
        throw std::runtime_error("Robot registration failed: No robot ID returned");
    }
}

void RobotStateController::start_session() {
    // Create session payload
    json payload = {
        {"anonymous", true}
    };

    // Send session creation request
    bool session_done = false;
    std::string session_error;

    web_client_->sendJsonPostAsync(
        "/api/robotsessions/",
        payload,
        {"Content-Type: application/json"},
        [this, &session_done, &session_error](const std::string& body, long http_code) {
            if (http_code >= 200 && http_code < 300) {
                try {
                    auto response = json::parse(body);
                    if (response.contains("id")) {
                        session_id_ = response["id"].get<std::string>();
                        RCLCPP_INFO(get_logger(), "Session created successfully. ID: %s", session_id_.c_str());
                    } else {
                        session_error = "No ID in session response";
                    }
                } catch (const std::exception& e) {
                    session_error = std::string("Failed to parse session response: ") + e.what();
                }
            } else {
                session_error = "HTTP " + std::to_string(http_code) + ": " + body;
            }
            session_done = true;
        }
    );

    // Wait for session creation to complete
    auto start_time = std::chrono::steady_clock::now();
    while (!session_done && std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!session_done) {
        throw std::runtime_error("Session creation timeout");
    }
    if (!session_error.empty()) {
        throw std::runtime_error("Session creation failed: " + session_error);
    }
    if (session_id_.empty()) {
        throw std::runtime_error("Session creation failed: No session ID returned");
    }
}

void RobotStateController::add_robot_to_session() {
    // Generate UUID for robot state entry
    uuid_t uuid;
    uuid_generate(uuid);
    char uuid_str[37];
    uuid_unparse(uuid, uuid_str);

    // Create state payload
    json payload = {
        {"id", std::string(uuid_str)},
        {"robotId", robot_id_},
        {"status", "waiting"},
        {"currentTask", "idle"},
        {"currentBehaviorId", nullptr},
        {"lastStatusChange", get_current_timestamp()},
        {"speechLog", ""}
    };

    // Send add robot request
    bool add_done = false;
    std::string add_error;

    std::string endpoint = "/api/robotsessions/" + session_id_ + "/robots";

    web_client_->sendJsonPostAsync(
        endpoint,
        payload,
        {"Content-Type: application/json"},
        [this, &add_done, &add_error](const std::string& body, long http_code) {
            if (http_code >= 200 && http_code < 300) {
                RCLCPP_INFO(get_logger(), "Robot added to session successfully");
            } else {
                add_error = "HTTP " + std::to_string(http_code) + ": " + body;
            }
            add_done = true;
        }
    );

    // Wait for add robot to complete
    auto start_time = std::chrono::steady_clock::now();
    while (!add_done && std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!add_done) {
        throw std::runtime_error("Add robot to session timeout");
    }
    if (!add_error.empty()) {
        throw std::runtime_error("Add robot to session failed: " + add_error);
    }
}

void RobotStateController::driver_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    try {
        // Parse incoming state message (expected format: JSON with status and task)
        auto state_data = json::parse(msg->data);

        if (state_data.contains("status")) {
            current_state_ = state_data["status"].get<std::string>();
        }
        if (state_data.contains("currentTask")) {
            current_task_ = state_data["currentTask"].get<std::string>();
        }

        RCLCPP_DEBUG(get_logger(), "State updated: %s / %s", current_state_.c_str(), current_task_.c_str());

    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to parse driver state message: %s", e.what());
    }
}

void RobotStateController::sync_state_to_backend() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Create state payload
    json state_payload = {
        {"status", current_state_},
        {"currentTask", current_task_},
        {"currentBehaviorId", nullptr},
        {"lastStatusChange", get_current_timestamp()},
        {"speechLog", ""}
    };

    std::string endpoint = "/api/robotsessions/" + session_id_ + "/robots/" + robot_id_ + "/state";

    web_client_->sendRequestAsync(
        "PUT",
        endpoint,
        state_payload.dump(),
        std::nullopt,
        {"Content-Type: application/json"},
        [this](const std::string&, long http_code) {
            if (http_code >= 200 && http_code < 300) {
                RCLCPP_DEBUG(get_logger(), "State synced to backend successfully");
            } else {
                RCLCPP_WARN(get_logger(), "Failed to sync state to backend: HTTP %ld", http_code);
            }
        }
    );
}

void RobotStateController::send_backend_request(
    const std::string& method,
    const std::string& endpoint,
    const std::string& payload) {

    web_client_->sendRequestAsync(
        method,
        endpoint,
        payload,
        std::nullopt,
        {"Content-Type: application/json"},
        [this, endpoint](const std::string&, long http_code) {
            if (http_code >= 200 && http_code < 300) {
                RCLCPP_DEBUG(get_logger(), "Backend request successful: %s (HTTP %ld)", endpoint.c_str(), http_code);
            } else {
                RCLCPP_WARN(get_logger(), "Backend request failed: %s (HTTP %ld)", endpoint.c_str(), http_code);
            }
        }
    );
}

void RobotStateController::backend_command_callback(const std::string& command) {
    // TODO: Implement command handling from C# backend
    // This will be called when backend sends commands to robot
    RCLCPP_INFO(get_logger(), "Received command from backend (not implemented): %s", command.c_str());
}

std::string RobotStateController::get_current_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t_now), "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << "Z";

    return oss.str();
}

std::string RobotStateController::extract_json_value(
    const std::string& response,
    const std::string& key) const {

    try {
        auto json_response = json::parse(response);
        if (json_response.contains(key)) {
            return json_response[key].get<std::string>();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to extract JSON value: %s", e.what());
    }
    return "";
}

std::string RobotStateController::get_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

std::string RobotStateController::get_task() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_task_;
}

std::string RobotStateController::get_robot_id() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return robot_id_;
}

std::string RobotStateController::get_session_id() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return session_id_;
}

} // namespace robot_state_controller
