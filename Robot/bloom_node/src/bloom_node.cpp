// launcher for bloom_node: composes StateManager and WebServiceClient

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "bloom_node/state_manager.h"
#include "bloom_node/web_service_client.h"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	// Create nodes
	auto state_mgr = std::make_shared<state_manager::StateManager>(rclcpp::NodeOptions());
	// WebServiceClient constructor expects (node_name, base_url, default_timeout_ms, max_retries)
	// Provide sensible defaults here or read them from a config/params.
	auto web_client = std::make_shared<web_service_client::WebServiceClient>(
		std::string("web_service_client"),
		std::string("http://localhost:5000"),
		5000,
		2
	);

	// Multi-threaded executor to run both nodes concurrently
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(state_mgr);
	executor.add_node(web_client);


	RCLCPP_INFO(state_mgr->get_logger(), "bloom_node composed: state_manager + web_service_client starting");
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
