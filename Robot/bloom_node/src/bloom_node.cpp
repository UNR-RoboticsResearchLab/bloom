// launcher for bloom_node: composes StateManager, WebServiceClient, and ConfigManager

#include <memory>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "bloom_node/state_manager.h"
#include "bloom_node/web_service_client.h"
#include "bloom_node/configuration_manager.h"
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
	auto config_mgr = std::make_shared<configuration_manager::ConfigurationManager>(node);

	fs::path dir = "./config";
	std::vector<fs::path> files;

	if (!fs::exists(dir) || !fs::is_directory(dir)) {
		RCLCPP_ERROR(node->get_logger(), "The provided path is not a directory or does not exist.\n");
		return 1;
	}

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

	auto state_mgr = std::make_shared<state_manager::StateManager>(rclcpp::NodeOptions());

	// WebServiceClient constructor expects (node_name, base_url, default_timeout_ms, max_retries)
	auto web_client = std::make_shared<web_service_client::WebServiceClient>(
		std::string("web_service_client"),
	  	std::string(config_mgr->get_string("base_url").value_or("")),
	  	5000,
	  	2
	);

	// Example: Start a session with a POST request to /api/robot/sessions
	nlohmann::json session_payload = {
		{"anonymous", false}
	};

	web_client->enableResponsePublisher("/status_response");

	auto session_future = web_client->sendRequestAsync(
		"POST",
		"/api/robotsessions/",
		session_payload.dump(),
		std::nullopt,
		{"Content-Type: application/json"},
		[web_client](const std::string &body, long http_code) {
			if (http_code >= 200 && http_code < 300) {
				RCLCPP_INFO(web_client->get_logger(), "Session started successfully (HTTP %ld)", http_code);
				RCLCPP_DEBUG(web_client->get_logger(), "Response: %s", body.c_str());
			} else {
				RCLCPP_WARN(web_client->get_logger(), "Failed to start session (HTTP %ld): %s", http_code, body.c_str());
			}
		}
	);



	// Multi-threaded executor to run nodes concurrently
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(state_mgr);
	executor.add_node(web_client);
	executor.add_node(config_mgr);


	// RCLCPP_INFO(state_mgr->get_logger(), "bloom_node composed: state_manager + web_service_client starting");
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
