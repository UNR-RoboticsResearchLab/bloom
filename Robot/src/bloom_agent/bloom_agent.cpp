
// launcher for bloom_agent: Robot state controller for Bloom system
// Manages robot state synchronization between ROS2 driver and C# backend

#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "bloom_node/web_service_client.h"
#include "bloom_node/configuration_manager.h"
#include "bloom_agent/robot_state_controller.h"

namespace fs = std::filesystem;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        // ====== Initialize configuration ======
        auto config_node = std::make_shared<rclcpp::Node>("config_loader");
        auto config_mgr = std::make_shared<configuration_manager::ConfigurationManager>(config_node);

        // Load configuration from file
        fs::path config_dir = "src/bloom_agent/config";

        if (!fs::exists(config_dir) || !fs::is_directory(config_dir)) {
            RCLCPP_ERROR(config_node->get_logger(), "Config directory not found: %s", config_dir.c_str());
            return 1;
        }

        // Find and load first config file
        std::vector<fs::path> config_files;
        for (const auto& entry : fs::directory_iterator(config_dir)) {
            if (fs::is_regular_file(entry)) {
                config_files.push_back(entry.path());
            }
        }
        std::sort(config_files.begin(), config_files.end());

        if (!config_files.empty()) {
            config_mgr->load_from_file(config_files.front().c_str());
            RCLCPP_INFO(config_node->get_logger(), "Loaded config: %s", config_files.front().c_str());
        } else {
            RCLCPP_WARN(config_node->get_logger(), "No config file found in %s", config_dir.c_str());
        }

        // ====== Initialize WebServiceClient ======
        std::string base_url = config_mgr->get_string("base_url").value_or("http://localhost:5000");
        auto web_client = std::make_shared<web_service_client::WebServiceClient>(
            "web_service_client",
            base_url,
            5000,  // timeout_ms
            5      // max_retries
        );
        RCLCPP_INFO(config_node->get_logger(), "WebServiceClient initialized with base_url: %s", base_url.c_str());

        // ====== Initialize RobotStateController ======
        auto controller = std::make_shared<robot_state_controller::RobotStateController>(
            web_client,
            config_mgr
        );

        // ====== Create multi-threaded executor ======
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(),
            2  // number of threads
        );

        executor.add_node(web_client);
        executor.add_node(controller);

        RCLCPP_INFO(config_node->get_logger(), "bloom_agent started successfully. Spinning executor...");
        executor.spin();

        RCLCPP_INFO(config_node->get_logger(), "bloom_agent shutdown");
        rclcpp::shutdown();
        return 0;

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("bloom_agent"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}