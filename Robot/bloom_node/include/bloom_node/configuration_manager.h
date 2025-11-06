#ifndef CONFIGURATION_MANAGER_H
#define CONFIGURATION_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace configuration_manager {

/// ConfigurationManager provides an (untested) thread-safe configuration store for
/// the robot. It can load simple key=value files and import parameters from a
/// provided ROS2 node. Values are stored as strings and basic typed getters
/// (string/int/double/bool) are provided.
/// 
class ConfigurationManager : public rclcpp::Node
{
public:
	using Ptr = std::shared_ptr<ConfigurationManager>;


	
	ConfigurationManager(const rclcpp::Node::SharedPtr& node);
	
	ConfigurationManager(const ConfigurationManager &) = delete;
	ConfigurationManager & operator=(const ConfigurationManager &) = delete;
	
	~ConfigurationManager() = default;

	// Load key=value pairs from a plain text file. Lines starting with '#'
	// are treated as comments. Existing keys are overwritten.
	// Returns true on success, false on IO error.
	bool load_from_file(const std::string & path);

	// Import all parameters from the associated ROS2 node (if present).
	//m Parameters will be stored using their full names as keys.
	void import_from_node();

	// Reload from file (convenience wrapper). Returns success status.
	bool reload_file(const std::string & path);

    // Save to file. returns success.
    bool save_to_file(const std::string & path) const;

	// Typed getters. If the key is missing, returns std::nullopt
	std::optional<std::string> get_string(const std::string & key) const;
	std::optional<int> get_int(const std::string & key) const;
	std::optional<double> get_double(const std::string & key) const;
	std::optional<bool> get_bool(const std::string & key) const;

	// Set or replace a key value (string)
	void set(const std::string & key, const std::string & value);

	// Return a snapshot copy of the key->value map
	std::unordered_map<std::string, std::string> snapshot() const;

private:

    void on_config_message(std_msgs::msg::String::SharedPtr msg);

	// Helper: parse a line of key=value
	static bool parse_line(const std::string & line, std::string & key, std::string & value);

	rclcpp::Node::SharedPtr node_;
	mutable std::mutex mutex_;
	std::unordered_map<std::string, std::string> store_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

} // namespace bloom_node


#endif