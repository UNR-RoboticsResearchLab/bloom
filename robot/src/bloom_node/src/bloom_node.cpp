// launcher for bloom_node: composes StateManager, WebServiceClient, 
// BehaviorCoordinator, LessonCoordinator, LessonPoller, and FeedbackPoller
// into a single executable

#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bloom_msgs/action/play_behavior.hpp"
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

	// Detect the outbound LAN IP on every boot and write it to the persistent config.
	// A UDP socket is opened but never sends data — connect() only sets the routing entry.
	{
		std::string detected_ip;
		int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (sock >= 0) {
			sockaddr_in dest{};
			dest.sin_family = AF_INET;
			dest.sin_port   = htons(80);
			::inet_pton(AF_INET, "8.8.8.8", &dest.sin_addr);
			if (::connect(sock, reinterpret_cast<sockaddr *>(&dest), sizeof(dest)) == 0) {
				sockaddr_in local{};
				socklen_t len = sizeof(local);
				if (::getsockname(sock, reinterpret_cast<sockaddr *>(&local), &len) == 0) {
					char buf[INET_ADDRSTRLEN];
					::inet_ntop(AF_INET, &local.sin_addr, buf, sizeof(buf));
					detected_ip = buf;
				}
			}
			::close(sock);
		}

		if (!detected_ip.empty()) {
			config_mgr->set("robot_ip_address", detected_ip);
			if (!persistent_config.empty())
				config_mgr->save_to_file(persistent_config.string());
			RCLCPP_INFO(node->get_logger(), "Local IP detected and saved: %s", detected_ip.c_str());
		} else {
			RCLCPP_WARN(node->get_logger(), "Could not detect local IP - using value from config");
		}
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

	// Action client to execute behaviors from coordinator queue via the
	// play_behavior action server (owned by whichever robot body node is
	// running, e.g. openhmi_blossom's sequence_player in the blsm_unr repo).
	using PlayBehavior = bloom_msgs::action::PlayBehavior;
	auto behavior_action_client = rclcpp_action::create_client<PlayBehavior>(node, "play_behavior");

	// Timer to process queued behaviors and execute them
	// Runs every 100ms to check if next high-priority behavior should execute.
	// get_next_behavior() only ever returns a name when it's safe to dispatch
	// (an empty execution slot, or an explicit interrupt), so no client-side
	// in-flight tracking is needed here -- the action server preempts on its
	// own, and behavior_completed() below is what frees the slot back up.
	auto behavior_execution_timer = node->create_wall_timer(
		std::chrono::milliseconds(100),
		[behavior_coord, behavior_action_client, node]() {
			// Get next behavior from priority queue
			std::string next_behavior = behavior_coord->get_next_behavior();

			if (next_behavior.empty()) return;

			if (!behavior_action_client->action_server_is_ready()) {
				RCLCPP_WARN(node->get_logger(),
					"play_behavior action server not available - dropping behavior: %s",
					next_behavior.c_str());
				behavior_coord->behavior_completed(next_behavior);
				return;
			}

			PlayBehavior::Goal goal;
			goal.behavior_name = next_behavior;

			rclcpp_action::Client<PlayBehavior>::SendGoalOptions options;
			options.goal_response_callback =
				[behavior_coord, node, next_behavior](
					const rclcpp_action::ClientGoalHandle<PlayBehavior>::SharedPtr & goal_handle) {
					if (!goal_handle) {
						RCLCPP_WARN(node->get_logger(),
							"Behavior rejected by play_behavior action server: %s", next_behavior.c_str());
						behavior_coord->behavior_completed(next_behavior);
					}
				};
			options.result_callback =
				[behavior_coord, node, next_behavior](
					const rclcpp_action::ClientGoalHandle<PlayBehavior>::WrappedResult & result) {
					switch (result.code) {
						case rclcpp_action::ResultCode::SUCCEEDED:
							RCLCPP_DEBUG(node->get_logger(), "Behavior completed: %s", next_behavior.c_str());
							break;
						case rclcpp_action::ResultCode::CANCELED:
							RCLCPP_INFO(node->get_logger(), "Behavior preempted: %s", next_behavior.c_str());
							break;
						case rclcpp_action::ResultCode::ABORTED:
						default:
							RCLCPP_WARN(node->get_logger(), "Behavior failed: %s", next_behavior.c_str());
							break;
					}
					behavior_coord->behavior_completed(next_behavior);
				};

			behavior_action_client->async_send_goal(goal, options);

			RCLCPP_DEBUG(node->get_logger(), "Dispatched queued behavior: %s (pending=%zu)",
				next_behavior.c_str(), behavior_coord->pending_count());
		}
	);

	web_client->enableResponsePublisher("/status_response");

	// Register and save credentials / login to get robot id to feed to sessionId
	std::string serial_number = config_mgr->get_string("robot_serial_number").value_or("");
	if (serial_number.empty()) {
		char hostname[256];
		if (gethostname(hostname, sizeof(hostname)) == 0) {
			serial_number = hostname;
		} else {
			serial_number = "unknown";
		}
	}

	nlohmann::json robotInfo = {
		{"name", config_mgr->get_string("robot_name").value_or("Unnamed Robot")},
		{"model", config_mgr->get_string("robot_model").value_or("Unknown Model")},
		{"serialNumber", serial_number},
		{"firmwareVersion", config_mgr->get_string("robot_firmware_version").value_or("N/A")},
		{"ipAddress", config_mgr->get_string("robot_ip_address").value_or("N/A")}
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
			"/api/robot/register",
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
					RCLCPP_WARN(web_client->get_logger(), 
						"Failed to register robot (HTTP %ld): %s", http_code, body.c_str());
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

	// Login with robot credentials to get a JWT for all subsequent requests.
	// Matches on Name + IPAddress — same fields LoginRobotAsync checks on the server.
	std::string jwt_token;
	auto do_login = [&]() {
		auto login_future = web_client->sendJsonPostAsync(
			"/api/robot/login",
			robotInfo,
			{"Content-Type: application/json"},
			[web_client, &jwt_token](const std::string &body, long http_code) {
				if (http_code >= 200 && http_code < 300) {
					try {
						auto response = nlohmann::json::parse(body);
						if (response.contains("token")) {
							jwt_token = response["token"].get<std::string>();
							web_client->setAuthHeader("Authorization: Bearer " + jwt_token);
							RCLCPP_INFO(web_client->get_logger(), "Robot authenticated - JWT acquired");
						} else {
							RCLCPP_WARN(web_client->get_logger(), "Login response missing 'token' field");
						}
					} catch (const std::exception &e) {
						RCLCPP_WARN(web_client->get_logger(), "Failed to parse login response: %s", e.what());
					}
				} else {
					RCLCPP_WARN(web_client->get_logger(),
						"Robot login failed (HTTP %ld): %s", http_code, body.c_str());
				}
			}
		);
		login_future.get();
	};

	do_login();
	if (jwt_token.empty()) {
		RCLCPP_ERROR(node->get_logger(),
			"Robot login failed — authenticated requests will be rejected by the backend");
	}

	// Example: Start a session with a POST request to /api/robot/sessions
	RCLCPP_INFO(node->get_logger(), "Robot ID before session creation: %s (empty=%s)",
		robotId.c_str(), robotId.empty() ? "true" : "false");

	nlohmann::json session_payload = {
		{"anonymous", false},
		{"robotId", robotId}
	};

	RCLCPP_INFO(node->get_logger(), "Session payload: %s", session_payload.dump().c_str());


	// Extract session ID from POST response
	std::string session_id;
	std::string pairing_code;
	auto session_future = web_client->sendRequestAsync(
		"POST",
		"/api/robotsession/",
		session_payload.dump(),
		std::nullopt,
		{"Content-Type: application/json"},
		[web_client, &session_id, &robotId, &pairing_code, config_mgr](const std::string &body, long http_code) {
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
			} else if (http_code == 404) {
				RCLCPP_WARN(web_client->get_logger(),
					"Cached robot ID not found in backend (HTTP 404) — clearing cached ID for re-registration");
				robotId.clear();
				config_mgr->set("robot_id", "");
			} else {
				RCLCPP_WARN(web_client->get_logger(), "Failed to start session (HTTP %ld): %s", http_code, body.c_str());
			}
		}
	);

	// Wait for session creation to complete
	session_future.get();

	// If session creation failed due to stale robot_id, re-register and retry
	if (session_id.empty() && robotId.empty()) {
		RCLCPP_INFO(node->get_logger(), "Re-registering robot after stale ID detected");
		
		auto rereg_future = web_client->sendJsonPostAsync(
			"/api/robot/register",
			robotInfo,
			{"Content-Type: application/json"},
			[web_client, &robotId, config_mgr](const std::string &body, long http_code) {
				if (http_code >= 200 && http_code < 300) {
					try {
						auto response = nlohmann::json::parse(body);
						if (response.contains("robot") && response["robot"].contains("id")) {
							robotId = response["robot"]["id"].get<std::string>();
							config_mgr->set("robot_id", robotId);
							RCLCPP_INFO(web_client->get_logger(), "Re-registered with new robot ID: %s", robotId.c_str());
						}
					} catch (const std::exception &e) {
						RCLCPP_WARN(web_client->get_logger(), "Failed to parse re-registration response: %s", e.what());
					}
				} else {
					RCLCPP_ERROR(web_client->get_logger(), "Re-registration failed (HTTP %ld): %s", http_code, body.c_str());
				}
			}
		);
		rereg_future.get();

		// Re-authenticate after re-registration so the session retry carries a valid JWT
		jwt_token.clear();
		do_login();

		// Retry session creation with new robot ID
		if (!robotId.empty()) {
			nlohmann::json retry_payload = {
				{"anonymous", false},
				{"robotId", robotId}
			};
			auto retry_future = web_client->sendRequestAsync(
				"POST",
				"/api/robotsession/",
				retry_payload.dump(),
				std::nullopt,
				{"Content-Type: application/json"},
				[web_client, &session_id, &pairing_code](const std::string &body, long http_code) {
					if (http_code >= 200 && http_code < 300) {
						try {
							auto response = nlohmann::json::parse(body);
							if (response.contains("id")) {
								session_id = response["id"].get<std::string>();
								RCLCPP_INFO(web_client->get_logger(), "Session created on retry: %s", session_id.c_str());
							}
							if (response.contains("sessionCode")) {
								pairing_code = response["sessionCode"].get<std::string>();
								web_client->publishSessionCode(pairing_code);
							}
						} catch (const std::exception &e) {
							RCLCPP_WARN(web_client->get_logger(), "Failed to parse retry session response: %s", e.what());
						}
					} else {
						RCLCPP_ERROR(web_client->get_logger(), "Session retry failed (HTTP %ld): %s", http_code, body.c_str());
					}
				}
			);
			retry_future.get();
		}
		if (session_id.empty()) {
			RCLCPP_ERROR(node->get_logger(), 
				"FATAL: Could not establish session with backend. "
				"Check that the backend is running and accessible at %s",
				config_mgr->get_string("base_url").value_or("unknown").c_str());
			// Continue anyway so other nodes keep running, but lesson functionality will be disabled
		}
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
	lesson_poller->set_pairing_code(pairing_code);
	feedback_poller->start_polling();  // Starts in inactive state, activated by LessonCoordinator during interactions

	// ====== JWT Refresh Timer ======
	// Refresh 5 minutes before the 30-minute token expiry so requests never hit an expired token.
	auto jwt_refresh_timer = node->create_wall_timer(
		std::chrono::minutes(25),
		[&do_login, node]() {
			RCLCPP_INFO(node->get_logger(), "Refreshing JWT token (25-minute interval)");
			do_login();
		}
	);

	// ====== Session Inactivity Reset Timer ======
	// If no lesson has been running for 5 minutes, end the current session and open a fresh one
	// so the robot is immediately re-pairable without a manual restart.
	auto last_lesson_active_time = std::make_shared<std::chrono::steady_clock::time_point>(
		std::chrono::steady_clock::now());

	// Keep the activity timestamp current whenever a lesson finishes.
	lesson_coord->set_completion_callback(
		[last_lesson_active_time, node](const std::string &lesson_id) {
			*last_lesson_active_time = std::chrono::steady_clock::now();
			RCLCPP_INFO(node->get_logger(), "Lesson %s completed — inactivity timer reset", lesson_id.c_str());
		}
	);

	auto session_inactivity_timer = node->create_wall_timer(
		std::chrono::seconds(60),
		[&session_id, &pairing_code, &robotId, lesson_coord, lesson_poller,
		 feedback_poller, web_client, last_lesson_active_time, node]() {

			if (session_id.empty()) return;

			// Reset the clock while a lesson is actively running.
			if (lesson_coord->is_lesson_running()) {
				*last_lesson_active_time = std::chrono::steady_clock::now();
				return;
			}

			auto elapsed = std::chrono::steady_clock::now() - *last_lesson_active_time;
			if (elapsed < std::chrono::minutes(5)) return;

			RCLCPP_INFO(node->get_logger(),
				"Session %s idle for 5 minutes — resetting session", session_id.c_str());

			// End the stale session.
			web_client->sendRequestAsync(
				"DELETE",
				"/api/robotsession/" + session_id,
				std::nullopt, std::nullopt, {},
				[web_client](const std::string &body, long http_code) {
					if (http_code >= 200 && http_code < 300)
						RCLCPP_INFO(web_client->get_logger(), "Stale session ended (HTTP %ld)", http_code);
					else
						RCLCPP_WARN(web_client->get_logger(), "End session returned HTTP %ld: %s", http_code, body.c_str());
				}
			).get();

			// Open a fresh session with the same robot.
			nlohmann::json new_session_payload = {{"anonymous", false}, {"robotId", robotId}};
			web_client->sendRequestAsync(
				"POST",
				"/api/robotsession/",
				new_session_payload.dump(),
				std::nullopt,
				{"Content-Type: application/json"},
				[&session_id, &pairing_code, lesson_poller, feedback_poller, lesson_coord, web_client, node](
					const std::string &body, long http_code) {
					if (http_code < 200 || http_code >= 300) {
						RCLCPP_ERROR(web_client->get_logger(),
							"Failed to create replacement session (HTTP %ld): %s", http_code, body.c_str());
						return;
					}
					try {
						auto response = nlohmann::json::parse(body);
						if (response.contains("id")) {
							session_id = response["id"].get<std::string>();
							lesson_poller->set_session_id(session_id);
							feedback_poller->set_session_id(session_id);
							lesson_coord->set_session_id(session_id);
							RCLCPP_INFO(node->get_logger(), "New session created: %s", session_id.c_str());
						}
						if (response.contains("sessionCode")) {
							pairing_code = response["sessionCode"].get<std::string>();
							web_client->publishSessionCode(pairing_code);
							lesson_poller->set_pairing_code(pairing_code);
							RCLCPP_INFO(node->get_logger(), "New session code: %s", pairing_code.c_str());
						}
					} catch (const std::exception &e) {
						RCLCPP_WARN(web_client->get_logger(), "Failed to parse new session response: %s", e.what());
					}
				}
			).get();

			*last_lesson_active_time = std::chrono::steady_clock::now();
		}
	);

	RCLCPP_INFO(node->get_logger(), "bloom_node started - state_manager + web_service_client + lesson_coordinator + lesson_poller + feedback_poller running");
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
