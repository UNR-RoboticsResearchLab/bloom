
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
    auto node = std::make_shared<rclcpp::Node>("bloom_agent");

    

}