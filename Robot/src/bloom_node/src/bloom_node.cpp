// launcher for bloom_node: composes StateManager, WebServiceClient, and ConfigManager

#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include "bloom_node/state_manager.h"
#include "bloom_node/web_service_client.h"
#include "bloom_node/configuration_manager.h"
#include "bloom_node/behavior_coordinator.h"
#include "bloom_node/lessson_coordinator.h"
#include "bloom_node/lesson_poller.h"
#include "bloom_node/json.hpp"

namespace fs = std::filesystem;

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

    // ====== declare node and init configuration params ======
    auto node = std::make_shared<rclcpp::Node>("bloom_node");
	


	// ====== Create nodes ======
	//config requires some config
	// todo: move to helper
	auto config_mgr = std::make_shared<bloom_node::ConfigurationManager>(node);

	// ====== Create BehaviorCoordinator ======
	auto behavior_coord = std::make_shared<bloom_node::BehaviorCoordinator>();

	// Configure exclusive behavior groups
	behavior_coord->set_exclusive_group("head_movement", {
		"look_left", "look_right", "look_up", "look_down", "nod", "shake"
	});

	behavior_coord->set_exclusive_group("emotions", {
		"happy", "sad", "excited", "calm", "neutral", "surprised"
	});

	behavior_coord->set_exclusive_group("speak", {
		"speaking", "listening"
	});


	RCLCPP_INFO(node->get_logger(), "BehaviorCoordinator initialized with exclusive groups");

	fs::path dir = "src/bloom_node/config";

	if (!fs::exists(dir) || !fs::is_directory(dir)) {
        RCLCPP_ERROR(node->get_logger(), "The provided path is not a directory or does not exist.\n");
        RCLCPP_ERROR(node->get_logger(), dir.c_str());
        return 1;
    }

    // Vector to store file paths
    std::vector<fs::path> files;

	// Loop through the directory and store all file paths
	for (const auto& entry : fs::directory_iterator(dir)) {
		if (fs::is_regular_file(entry)) {
			files.push_back(entry.path());
		}
	}
	std::sort(files.begin(), files.end());

	if (!files.empty()) {
		config_mgr->load_from_file(files.front().c_str());
	}

	auto state_mgr = std::make_shared<bloom_node::StateManager>(rclcpp::NodeOptions());

	// WebServiceClient constructor expects (node_name, base_url, default_timeout_ms, max_retries)
	auto web_client = std::make_shared<bloom_node::WebServiceClient>(
		std::string("web_service_client"),
	  	std::string(config_mgr->get_string("base_url").value_or("")),
	  	5000,
	  	2
	);

	// ====== Setup Behavior Request Handler ======
	// Subscribe to behavior requests on /behavior_request topic
	// Message format: "behavior_name" or "behavior_name:priority:interrupt"
	// Examples: "happy", "nod:10:true", "speaking:5:false"

	auto behavior_request_sub = node->create_subscription<std_msgs::msg::String>(
		"behavior_request", 10,
		[behavior_coord, node](const std_msgs::msg::String::SharedPtr msg) {
			if (!msg || msg->data.empty()) return;

			std::string behavior = msg->data;
			int priority = 0;
			bool interrupt = false;

			// Parse format: "behavior_name:priority:interrupt"
			size_t colon1 = behavior.find(':');
			if (colon1 != std::string::npos) {
				std::string name = behavior.substr(0, colon1);
				size_t colon2 = behavior.find(':', colon1 + 1);

				if (colon2 != std::string::npos) {
					try {
						priority = std::stoi(behavior.substr(colon1 + 1, colon2 - colon1 - 1));
						interrupt = (behavior.substr(colon2 + 1) == "true");
						behavior = name;
					} catch (const std::exception &e) {
						RCLCPP_WARN(node->get_logger(), "Failed to parse behavior request: %s", msg->data.c_str());
						return;
					}
				}
			}

			behavior_coord->request_behavior(behavior, priority, interrupt);
			RCLCPP_DEBUG(node->get_logger(),
				"Behavior requested: %s (priority=%d, interrupt=%s, pending=%zu)",
				behavior.c_str(), priority, interrupt ? "yes" : "no",
				behavior_coord->pending_count());
		}
	);

	// Publisher to execute behaviors from coordinator queue
	auto behavior_execution_pub = node->create_publisher<std_msgs::msg::String>(
		"play_sequence", 10);

	// Timer to process queued behaviors and execute them
	// Runs every 100ms to check if next high-priority behavior should execute
	auto behavior_execution_timer = node->create_wall_timer(
		std::chrono::milliseconds(100),
		[behavior_coord, behavior_execution_pub, node]() {
			// Get next behavior from priority queue
			std::string next_behavior = behavior_coord->get_next_behavior();

			if (!next_behavior.empty()) {
				// Publish to behavior execution topic
				auto msg = std::make_shared<std_msgs::msg::String>();
				msg->data = next_behavior;
				behavior_execution_pub->publish(*msg);

				RCLCPP_DEBUG(node->get_logger(), "Executed queued behavior: %s (pending=%zu)",
					next_behavior.c_str(), behavior_coord->pending_count());
			}
		}
	);

	// Example: Start a session with a POST request to /api/robot/sessions
	nlohmann::json session_payload = {
		{"anonymous", false}
	};

	web_client->enableResponsePublisher("/status_response");

	// Extract session ID from POST response
	std::string session_id;
	auto session_future = web_client->sendRequestAsync(
		"POST",
		"/api/robotsessions/",
		session_payload.dump(),
		std::nullopt,
		{"Content-Type: application/json"},
		[web_client, &session_id](const std::string &body, long http_code) {
			if (http_code >= 200 && http_code < 300) {
				RCLCPP_INFO(web_client->get_logger(), "Session started successfully (HTTP %ld)", http_code);
				RCLCPP_INFO(web_client->get_logger(), "Response: %s", body.c_str());

				// Parse session_id from response JSON
				try {
					auto response = nlohmann::json::parse(body);
					if (response.contains("id")) {
						session_id = response["id"].get<std::string>();
						RCLCPP_INFO(web_client->get_logger(), "Session ID: %s", session_id.c_str());
					}
				} catch (const std::exception &e) {
					RCLCPP_WARN(web_client->get_logger(), "Failed to parse session ID from response: %s", e.what());
				}
			} else {
				RCLCPP_WARN(web_client->get_logger(), "Failed to start session (HTTP %ld): %s", http_code, body.c_str());
			}
		}
	);

	// Wait for session creation to complete
	session_future.get();

	// Create LessonCoordinator
	auto lesson_coord = std::make_shared<bloom_node::LessonCoordinator>(behavior_coord, web_client, state_mgr);
	RCLCPP_INFO(node->get_logger(), "LessonCoordinator created");

	// Create LessonPoller for backend-driven lesson polling (7 second interval)
	auto lesson_poller = std::make_shared<bloom_node::LessonPoller>(
		web_client,
		lesson_coord,
		session_id,
		7000  // Poll every 7 seconds
	);
	RCLCPP_INFO(node->get_logger(), "LessonPoller created with session_id: %s", session_id.c_str());


	// Multi-threaded executor to run nodes concurrently
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);  // Add main node for behavior_request subscription and timer
	executor.add_node(state_mgr);
	executor.add_node(web_client);
	executor.add_node(config_mgr);
	executor.add_node(lesson_coord);
	executor.add_node(lesson_poller);

	// Start the lesson polling loop
	lesson_poller->start_polling();

	RCLCPP_INFO(node->get_logger(), "bloom_node started - state_manager + web_service_client + lesson_coordinator + lesson_poller running");
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
