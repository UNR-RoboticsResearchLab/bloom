#ifndef WEB_SERVICE_CLIENT_H
#define WEB_SERVICE_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <curl/curl.h>
#include "bloom_node/json.hpp"

#include <string>
#include <functional>
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <chrono>
#include <optional>


namespace web_service_client
{

using json = nlohmann::json;
using namespace std::chrono_literals;

class WebServiceClient : public rclcpp::Node {

public:


    using ResponseCallback = std::function<void(const std::string &body, long http_code)>;

    WebServiceClient(
        const std::string &node_name,
        const std::string &base_url,
        int default_timeout_ms = 5000,
        int max_retries = 2
    );

    ~WebServiceClient() override;

    std::string pollStatus();

    std::string getSession();

    std::string getLesson();

    std::string getUser();

    bool postStatus();

    // Asynchronous request - returns std::future of same pair
    std::future<std::pair<std::string, long>> sendGetAsync(
        const std::string &path,
        const std::optional<std::string> &query = std::nullopt,
        const std::vector<std::string> &headers = {},
        ResponseCallback on_response = nullptr);

    std::future<std::pair<std::string, long>> sendPostAsync(
        const std::string &path,
        const std::string &body,
        const std::vector<std::string> &headers = {},
        ResponseCallback on_response = nullptr);

    // Convenience: send JSON POST (body will be serialized from nlohmann::json)
    std::future<std::pair<std::string, long>> sendJsonPostAsync(
        const std::string &path,
        const json &payload,
        const std::vector<std::string> &headers = {},
        ResponseCallback on_response = nullptr);

    // Publish the raw response body on a topic (std_msgs::msg::String)
    // Topic name default: "web_service/response"
    void enableResponsePublisher(const std::string &topic_name = "web_service/response");

    // Set an optional authentication header (e.g. "Authorization: Bearer ...")
    void setAuthHeader(const std::string &auth_header);


private:

    std::string ipAddr;
    // Helpers for curl
    static size_t writeCallback(char *ptr, size_t size, size_t nmemb, void *userdata);

    std::pair<std::string, long> performRequest(
        const std::string &method,
        const std::string &url,
        const std::string *body,
        const std::vector<std::string> &headers,
        int timeout_ms,
        int retries);

    std::string buildUrl(const std::string &path, const std::optional<std::string> &query);

    // Members
    std::string base_url_;
    int default_timeout_ms_;
    int max_retries_;
    std::string auth_header_;  // optional auth header (prefixed exactly as desired)
    
    std::mutex curl_init_mutex_;  // protects global curl init/cleanup if needed
    std::atomic<bool> running_{true};
        
    // ROS publishers / services
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
    bool publish_responses_{false};

    // A simple ROS service to trigger a GET to a given path provided via parameter
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;

    // Internal worker pool uses std::async; no explicit pool here for simplicity

    // Logging helper
    rclcpp::Logger logger_ = this->get_logger();

};

} // namespace web_service_client

#endif