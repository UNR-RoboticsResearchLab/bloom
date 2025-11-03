#include "bloom_node/configuration_manager.h"

using namespace configuration_manager;

ConfigurationManager::ConfigurationManager(const rclcpp::Node::SharedPtr& node)
  	: rclcpp::Node(node->get_name()),
	node_(node)
{
  	// If a node was supplied, subscribe to a config update topic on that node.
  	if (node_) {
    	sub_ = node_->create_subscription<std_msgs::msg::String>(
      		"/config/update",
      		10,
      		std::bind(&ConfigurationManager::on_config_message, this, std::placeholders::_1)
    	);
      RCLCPP_INFO(this->get_logger(), "ConfigurationManager node constructed");
  }

}

bool ConfigurationManager::load_from_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in.is_open()) return false;

  std::string line;
  std::unordered_map<std::string, std::string> tmp;
  while (std::getline(in, line)) {
    // Trim whitespace
    auto start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) continue;
    if (line[start] == '#') continue;
    std::string key, value;
    if (parse_line(line, key, value)) {
      tmp[key] = value;
    }
  }

  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto &p : tmp) store_[p.first] = p.second;
  }
  return true;
}

void ConfigurationManager::import_from_node()
{
  if (!node_) return;
  // list all parameters (top-level, depth 1)
  auto result = node_->list_parameters({}, 10);
  for (const auto &name : result.names) {
    rclcpp::Parameter p;
    if (node_->get_parameter(name, p)) {
      std::string sval;
      switch (p.get_type()) {
        case rclcpp::ParameterType::PARAMETER_STRING:
          sval = p.as_string(); break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          sval = std::to_string(p.as_int()); break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          sval = std::to_string(p.as_double()); break;
        case rclcpp::ParameterType::PARAMETER_BOOL:
          sval = p.as_bool() ? "true" : "false"; break;
        default:
          sval = ""; break;
      }
      if (!sval.empty()) {
        std::lock_guard<std::mutex> lk(mutex_);
        store_[name] = sval;
      }
    }
  }
}

bool ConfigurationManager::reload_file(const std::string & path)
{
  // Clear then load
  {
    std::lock_guard<std::mutex> lk(mutex_);
    store_.clear();
  }
  return load_from_file(path);
}

bool ConfigurationManager::save_to_file(const std::string & path) const
{
	std::scoped_lock lock(mutex_);

  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open config file for write: %s", path.c_str());
    } else {
      std::cerr << "Failed to open config file for write: " << path << std::endl;
    }
    return false;
  }

	for (const auto & [key, value] : store_) {
		out << key << "=" << value << "\n";
	}

	out.close();
  if (!out) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "Error occurred while writing to file: %s", path.c_str());
    } else {
      std::cerr << "Error occurred while writing to file: " << path << std::endl;
    }
    return false;
  }

  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "Configuration saved to %s", path.c_str());
  } else {
    std::cout << "Configuration saved to " << path << std::endl;
  }
	return true;
}

std::optional<std::string> ConfigurationManager::get_string(const std::string & key) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = store_.find(key);
  if (it == store_.end()) return std::nullopt;
  return it->second;
}

std::optional<int> ConfigurationManager::get_int(const std::string & key) const
{
  auto s = get_string(key);
  if (!s) return std::nullopt;
  try {
    return std::stoi(*s);
  } catch (...) { return std::nullopt; }
}

std::optional<double> ConfigurationManager::get_double(const std::string & key) const
{
  auto s = get_string(key);
  if (!s) return std::nullopt;
  try { return std::stod(*s); } catch (...) { return std::nullopt; }
}

std::optional<bool> ConfigurationManager::get_bool(const std::string & key) const
{
  auto s = get_string(key);
  if (!s) return std::nullopt;
  std::string v = *s;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  if (v == "true" || v == "1" || v == "yes") return true;
  if (v == "false" || v == "0" || v == "no") return false;
  return std::nullopt;
}

void ConfigurationManager::set(const std::string & key, const std::string & value)
{
  std::lock_guard<std::mutex> lk(mutex_);
  store_[key] = value;
}

std::unordered_map<std::string, std::string> ConfigurationManager::snapshot() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return store_;
}

bool ConfigurationManager::parse_line(const std::string & line, std::string & key, std::string & value)
{
  auto pos = line.find('=');
  if (pos == std::string::npos) return false;
  key = line.substr(0, pos);
  value = line.substr(pos + 1);
  // trim both
  auto trim = [](std::string & s) {
    auto l = s.find_first_not_of(" \t\r\n");
    auto r = s.find_last_not_of(" \t\r\n");
    if (l == std::string::npos) { s.clear(); return; }
    s = s.substr(l, r - l + 1);
  };
  trim(key); trim(value);
  return !key.empty();
}


void ConfigurationManager::on_config_message(const std_msgs::msg::String::SharedPtr msg)
{
    std::string key, value;
  if (parse_line(msg->data, key, value))
  {
    set(key, value);
    if (node_) RCLCPP_INFO(node_->get_logger(), "Config updated: %s=%s", key.c_str(), value.c_str());
    else std::cout << "Config updated: " << key << "=" << value << std::endl;
  }
  else
  {
    if (node_) RCLCPP_WARN(node_->get_logger(), "Invalid config message: '%s'", msg->data.c_str());
    else std::cerr << "Invalid config message: '" << msg->data << "'" << std::endl;
  }
}



#ifdef CONFIG_MANAGER_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("configuration_manager");
  bloom_node::ConfigurationManager cfg(node);

  node->declare

  if (argc > 1) {
    const std::string path = argv[1];
    if (!cfg.load_from_file(path)) {
      std::cerr << "Failed to load config file: " << path << std::endl;
    }
  }

  // Import any ROS params available on the node
  cfg.import_from_node();

  auto snap = cfg.snapshot();
  for (const auto &p : snap) {
    std::cout << p.first << "=" << p.second << std::endl;
  }

  rclcpp::spin();

  rclcpp::shutdown();
  return 0;
}
#endif
