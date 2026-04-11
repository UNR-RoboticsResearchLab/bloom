#ifndef WEB_SERVICE_CLIENT_H
#define WEB_SERVICE_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <curl/curl.h>
#include "bloom_node/json.hpp"
#include "bloom_node/thread_pool.h"
#include "bloom_node/curl_pool.h"

#include <string>
#include <functional>
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <chrono>
#include <optional>
#include <memory>

namespace bloom_node
{

    using json = nlohmann::json;
    using namespace std::chrono_literals;

    /**
     * Data Transfer Objects for Lesson API communication
     */

    struct LessonProgressUpdate {
        std::string current_step_id;
        int completed_steps;
        std::string status;  // "InProgress", "Completed", "Paused", "Failed"

        json to_json() const {
            json j;
            j["currentStepId"] = current_step_id;
            j["completedSteps"] = completed_steps;
            j["status"] = status;
            return j;
        }
    };

    struct StudentInteractionLog {
        std::string current_step_id;
        std::string interaction_type;  // "Response", "Question", "Timeout", "Fallback"
        std::string student_response;
        bool is_correct;
        int response_time_ms;

        json to_json() const {
            json j;
            j["stepId"] = step_id;
            j["interactionType"] = interaction_type;
            j["studentResponse"] = student_response;
            j["isCorrect"] = is_correct;
            j["responseTimeMs"] = response_time_ms;
            return j;
        }
    };

/**
 * Thread-safe HTTP client with connection pooling and bounded thread pool.
 * Provides async HTTP operations (GET, POST, PUT, DELETE, PATCH) with configurable
 * timeouts, retries, and SSL verification. Still requires JWT token management to be implemented.
 */
class WebServiceClient : public rclcpp::Node {

public:

    using ResponseCallback = std::function<void(const std::string &body, long http_code)>;

    /// Construct with node name, base URL, and optional thread/connection pool sizes
    WebServiceClient(
        const std::string &node_name,
        const std::string &base_url,
        int default_timeout_ms = 5000,
        int max_retries = 2,
        size_t thread_pool_size = 4,
        size_t curl_pool_size = 4,
        bool verify_ssl = true
    );

    /// Construct without explicit node name
    WebServiceClient(
        const std::string &base_url,
        int default_timeout_ms = 5000,
        int max_retries = 2,
        size_t thread_pool_size = 4,
        size_t curl_pool_size = 4,
        bool verify_ssl = true
    );

    ~WebServiceClient() override;

    // Non-copyable, non-movable
    WebServiceClient(const WebServiceClient &) = delete;
    WebServiceClient & operator=(const WebServiceClient &) = delete;
    WebServiceClient(WebServiceClient &&) = delete;
    WebServiceClient & operator=(WebServiceClient &&) = delete;

    // Asynchronous HTTP requests
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

    std::future<std::pair<std::string, long>> sendJsonPostAsync(
        const std::string &path,
        const json &payload,
        const std::vector<std::string> &headers = {},
        ResponseCallback on_response = nullptr);

    std::future<std::pair<std::string, long>> sendRequestAsync(
        const std::string &method,
        const std::string &path,
        const std::optional<std::string> &body = std::nullopt,
        const std::optional<std::string> &query = std::nullopt,
        const std::vector<std::string> &headers = {},
        ResponseCallback on_response = nullptr);

    // Configure SSL verification (true = verify peer certs, false = skip verification)
    void setVerifySSL(bool verify) { verify_ssl_ = verify; }

    // Publish HTTP responses to a ROS topic
    void enableResponsePublisher(const std::string &topic_name = "web_service/response");

    // Set authentication header (e.g., "Authorization: Bearer <token>")
    void setAuthHeader(const std::string &auth_header);

    // Publish session pairing code to face display
    void publishSessionCode(const std::string &session_code);

    // Get performance statistics
    size_t getThreadPoolQueueSize() const;
    size_t getCurlPoolAvailable() const;

private:

    // Static callback for CURL write operations
    static size_t writeCallback(char *ptr, size_t size, size_t nmemb, void *userdata);

    // Synchronous request execution (called by async methods via thread pool)
    std::pair<std::string, long> performRequest(
        const std::string &method,
        const std::string &url,
        const std::string *body,
        const std::vector<std::string> &headers,
        int timeout_ms,
        int retries);

    // Build full URL from path and query parameters
    std::string buildUrl(const std::string &path, const std::optional<std::string> &query);

    // Members
    std::string base_url_;
    int default_timeout_ms_;
    int max_retries_;
    bool verify_ssl_;
    std::string auth_header_;

    std::atomic<bool> running_{true};

    // Thread and connection management
    std::unique_ptr<bloom_node::ThreadPool> thread_pool_;
    std::unique_ptr<bloom_node::CurlPool> curl_pool_;

    // ROS publishers / services
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr session_code_pub_;
    bool publish_responses_{false};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
};

} // namespace web_service_client

#endif