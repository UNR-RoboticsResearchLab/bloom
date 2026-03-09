// launcher for bloom_node: composes StateManager, WebServiceClient, 
// BehaviorCoordinator, LessonCoordinator, LessonPoller, and FeedbackPoller
// into a single executable

#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bloom_node/state_manager.h"
#include "bloom_node/web_service_client.h"
#include "bloom_node/configuration_manager.h"
#include "bloom_node/behavior_coordinator.h"
#include "bloom_node/lessson_coordinator.h"
#include "bloom_node/lesson_poller.h"
#include "bloom_node/feedback_poller.h"
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

	fs::path dir = ament_index_cpp::get_package_share_directory("bloom_node") + "/config";

	if (!fs::exists(dir) || !fs::is_directory(dir)) {
		RCLCPP_WARN(node->get_logger(), "Config directory not found at %s, continuing without config", dir.c_str());
		RCLCPP_INFO(node->get_logger(), "Using default configuration");
		dir = "";
	}

	if (!fs::exists(dir) || !fs::is_directory(dir)) {
        RCLCPP_WARN(node->get_logger(), "Config directory not found at %s, continuing without config", dir.c_str());
        RCLCPP_INFO(node->get_logger(), "Using default configuration");
    }

    // Vector to store file paths
    std::vector<fs::path> files;

	// Loop through the directory and store all file paths
	if (dir.empty()) goto skip_config;
	for (const auto& entry : fs::directory_iterator(dir)) {
		if (fs::is_regular_file(entry)) {
			files.push_back(entry.path());
		}
	}
	skip_config:;
	std::sort(files.begin(), files.end());

	if (!files.empty()) {
		config_mgr->load_from_file(files.front().c_str());
	}

	// Load from (and save to) a persistent user config that survives colcon builds.
	// Values here override the install-dir config (e.g. robot_id persists across builds).
	fs::path persistent_config;
	if (const char* home = std::getenv("HOME")) {
		persistent_config = fs::path(home) / ".bloom" / "robot.cfg";
		fs::create_directories(persistent_config.parent_path());
		config_mgr->load_from_file(persistent_config.string());
	} else {
		RCLCPP_WARN(node->get_logger(), "HOME not set, persistent config unavailable");
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

	web_client->enableResponsePublisher("/status_response");

	// Register and save credentials / login to get robot id to feed to sessionId
	nlohmann::json robotInfo = {

		{"name", config_mgr->get_string("robot_name").value_or("Unnamed Robot")},
		{"model", config_mgr->get_string("robot_model").value_or("Unknown Model")},
		{"firmwareversion", config_mgr->get_string("robot_firmware_version").value_or("N/A")},
		{"ipaddress", config_mgr->get_string("robot_ip_address").value_or("N/A")}
	};

	std::string robotId;

	if (config_mgr->get_string("robot_id").has_value()) {
		robotId = config_mgr->get_string("robot_id").value();
		RCLCPP_INFO(node->get_logger(), "Using existing robot ID from config: %s", robotId.c_str());
	} else {
		RCLCPP_INFO(node->get_logger(), "No existing robot ID found in config, registering new robot");
	}

	if (robotId.empty())
	{
		auto registration_future = web_client->sendJsonPostAsync(
			"/api/robots/register",
			robotInfo,
			{"Content-Type: application/json"},
			[web_client, &robotId](const std::string &body, long http_code) {
				if (http_code >= 200 && http_code < 300) {
					RCLCPP_INFO(web_client->get_logger(), "Robot registered successfully (HTTP %ld)", http_code);
					RCLCPP_INFO(web_client->get_logger(), "Response: %s", body.c_str());
					// Parse robotId from response JSON
					try {
						auto response = nlohmann::json::parse(body);
						if (response.contains("robot") && response["robot"].contains("id")) {
							robotId = response["robot"]["id"].get<std::string>();
							RCLCPP_INFO(web_client->get_logger(), "Robot ID: %s", robotId.c_str());
						}
					} catch (const std::exception &e) {
						RCLCPP_WARN(web_client->get_logger(), "Failed to parse robot ID from response: %s", e.what());
					}

				} else {
					RCLCPP_WARN(web_client->get_logger(), "Failed to register robot (HTTP %ld): %s", http_code, body.c_str());
				}
			}
		);
		// Wait for registration to complete before proceeding
		registration_future.get();
		
		// save robot id
		if (!robotId.empty()) {
			RCLCPP_INFO(node->get_logger(), "Saving new robot ID to config: %s", robotId.c_str());
			config_mgr->set("robot_id", robotId);
		}
	}
	



	// Example: Start a session with a POST request to /api/robot/sessions
	RCLCPP_INFO(node->get_logger(), "Robot ID before session creation: %s (empty=%s)",
		robotId.c_str(), robotId.empty() ? "true" : "false");

	nlohmann::json session_payload = {
		{"anonymous", false},
		{"robot_id", robotId}
	};

	RCLCPP_INFO(node->get_logger(), "Session payload: %s", session_payload.dump().c_str());


	// Extract session ID from POST response
	std::string session_id;
	std::string pairing_code;
	auto session_future = web_client->sendRequestAsync(
		"POST",
		"/api/robotsessions/",
		session_payload.dump(),
		std::nullopt,
		{"Content-Type: application/json"},
		[web_client, &session_id, &robotId, &pairing_code](const std::string &body, long http_code) {
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
					if (response.contains("sessionCode")) {
						pairing_code = response["sessionCode"].get<std::string>();
						RCLCPP_INFO(web_client->get_logger(), "Session Code: %s", pairing_code.c_str());
						web_client->publishSessionCode(pairing_code);
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

	// Republish session code every 2s so face_node catches it after its delayed startup
	if (!pairing_code.empty()) {
		auto session_code_timer = node->create_wall_timer(
			std::chrono::milliseconds(2000),
			[web_client, pairing_code]() {
				web_client->publishSessionCode(pairing_code);
			}
		);
	}

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

	// Create FeedbackPoller for SLP feedback polling (1 second interval)
	auto feedback_poller = std::make_shared<bloom_node::FeedbackPoller>(
		web_client,
		session_id,
		300  // Poll every 300 milliseconds
	);
	RCLCPP_INFO(node->get_logger(), "FeedbackPoller created with session_id: %s", session_id.c_str());

	// Register FeedbackPoller with LessonCoordinator to control polling during interactions
	lesson_coord->set_feedback_poller(feedback_poller);

	// Multi-threaded executor to run nodes concurrently
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);  // Add main node for behavior_request subscription and timer
	executor.add_node(state_mgr);
	executor.add_node(web_client);
	executor.add_node(config_mgr);
	executor.add_node(lesson_coord);
	executor.add_node(lesson_poller);
	executor.add_node(feedback_poller);

	// Start the lesson and feedback polling loops
	lesson_poller->start_polling();
	feedback_poller->start_polling();  // Starts in inactive state, activated by LessonCoordinator during interactions

	RCLCPP_INFO(node->get_logger(), "bloom_node started - state_manager + web_service_client + lesson_coordinator + lesson_poller + feedback_poller running");
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
