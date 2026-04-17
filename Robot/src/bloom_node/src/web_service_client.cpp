#include "bloom_node/web_service_client.h"

namespace web_service_client {

WebServiceClient::WebServiceClient(
    const std::string &node_name,
    const std::string &base_url,
    int default_timeout_ms,
    int max_retries,
    size_t thread_pool_size,
    size_t curl_pool_size,
    bool verify_ssl
) : Node(node_name),
    base_url_(base_url),
    default_timeout_ms_(default_timeout_ms),
    max_retries_(max_retries),
    verify_ssl_(verify_ssl),
    thread_pool_(std::make_unique<bloom_node::ThreadPool>(thread_pool_size)),
    curl_pool_(std::make_unique<bloom_node::CurlPool>(curl_pool_size))
{
    // Initialize libcurl globally
    curl_global_init(CURL_GLOBAL_DEFAULT);

    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "get_status",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            this->sendPostAsync("status", "", {},
                [res](const std::string &/*body*/, long code) mutable {
                    res->success = (code >= 200 && code < 300);
                    res->message = "HTTP code: " + std::to_string(code);
                });
        });

    RCLCPP_INFO(this->get_logger(),
        "WebServiceClient initialized: url=%s, threads=%zu, ssl_verify=%s",
        base_url_.c_str(), thread_pool_size, verify_ssl ? "true" : "false");
}

WebServiceClient::WebServiceClient(
    const std::string &base_url,
    int default_timeout_ms,
    int max_retries,
    size_t thread_pool_size,
    size_t curl_pool_size,
    bool verify_ssl
) : WebServiceClient("web_service_client", base_url, default_timeout_ms, max_retries,
                     thread_pool_size, curl_pool_size, verify_ssl) {}

WebServiceClient::~WebServiceClient() {
    running_.store(false);
    // Thread pool and CURL pool destructors handle cleanup automatically
    curl_global_cleanup();
}

size_t WebServiceClient::writeCallback(char *ptr, size_t size, size_t nmemb, void *userdata)
{
  if (userdata)
  {
    std::string *s = static_cast<std::string *>(userdata);
    s->append(ptr, size * nmemb);
    return size * nmemb;
  }
  return 0;
}

std::string WebServiceClient::buildUrl(
  const std::string &path,
  const std::optional<std::string> &query)
{
  std::string url = base_url_;
  if (!path.empty())
  {
    if (url.back() == '/' && path.front() == '/')
      url.pop_back();
    else if (url.back() != '/' && path.front() != '/')
      url.push_back('/');
    url += path;
  }
  if (query && !query->empty())
  {
    url += "?";
    url += *query;
  }
  return url;
}

std::pair<std::string, long> WebServiceClient::performRequest(
    const std::string &method,
    const std::string &url,
    const std::string *body,
    const std::vector<std::string> &headers,
    int timeout_ms,
    int retries)
{
    // Acquire a CURL handle from the pool
    CURL *curl = curl_pool_->acquire();
    if (!curl) {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire CURL handle from pool");
        return {"", 0};
    }

    std::string response_data;
    struct curl_slist *hdrs = nullptr;

    try {
        // Merge headers with auth
        for (const auto &h : headers) {
            hdrs = curl_slist_append(hdrs, h.c_str());
        }
        if (!auth_header_.empty()) {
            hdrs = curl_slist_append(hdrs, auth_header_.c_str());
        }

        // Configure CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &WebServiceClient::writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, (long)timeout_ms);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, (long)(timeout_ms / 2));

        // SSL verification configuration
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, verify_ssl_ ? 1L : 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, verify_ssl_ ? 2L : 0L);

        if (hdrs) {
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
        }

        // Set HTTP method and body
        if (method == "POST") {
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            if (body) {
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->c_str());
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)body->size());
            }
        } else if (method == "PUT") {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
            if (body) {
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->c_str());
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)body->size());
            }
        } else if (method == "DELETE") {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
            if (body) {
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->c_str());
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)body->size());
            }
        } else if (method == "PATCH") {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PATCH");
            if (body) {
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->c_str());
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)body->size());
            }
        } else if (method == "HEAD") {
            curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
        } else if (method == "OPTIONS") {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "OPTIONS");
        }
        // GET is the default

        // Perform request with retries
        long http_code = 0;
        CURLcode response;
        int attempt = 0;

        do {
            response = curl_easy_perform(curl);

            if (response == CURLE_OK) {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
                break;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "curl_easy_perform() failed: %s (attempt %d/%d)",
                    curl_easy_strerror(response), attempt + 1, retries + 1);
                if (attempt < retries && running_.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200 * (attempt + 1)));
                }
            }
            attempt++;
        } while (attempt <= retries && running_.load());

        // Cleanup headers
        if (hdrs) {
            curl_slist_free_all(hdrs);
        }

        // Publish response if enabled
        if (publish_responses_ && response_pub_) {
            auto msg = std_msgs::msg::String();
            msg.data = response_data;
            response_pub_->publish(msg);
        }

        return {response_data, http_code};

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in performRequest: %s", e.what());
        if (hdrs) curl_slist_free_all(hdrs);
        curl_pool_->release(curl);
        return {"", 0};
    }

    // Return handle to pool
    curl_pool_->release(curl);
    return {"", 0};
}


std::future<std::pair<std::string, long>> WebServiceClient::sendGetAsync(
    const std::string &path,
    const std::optional<std::string> &query,
    const std::vector<std::string> &headers,
    ResponseCallback on_response)
{
    auto url = buildUrl(path, query);
    return thread_pool_->enqueue([this, url, headers, on_response]() {
        auto res = this->performRequest("GET", url, nullptr, headers,
                                       this->default_timeout_ms_, this->max_retries_);
        if (on_response) {
            on_response(res.first, res.second);
        }
        return res;
    });
}

std::future<std::pair<std::string, long>> WebServiceClient::sendPostAsync(
    const std::string &path,
    const std::string &body,
    const std::vector<std::string> &headers,
    ResponseCallback on_response)
{
    auto url = buildUrl(path, std::nullopt);
    return thread_pool_->enqueue([this, url, body, headers, on_response]() {
        auto res = this->performRequest("POST", url, &body, headers,
                                       this->default_timeout_ms_, this->max_retries_);
        if (on_response) {
            on_response(res.first, res.second);
        }
        return res;
    });
}

std::future<std::pair<std::string, long>> WebServiceClient::sendJsonPostAsync(
    const std::string &path,
    const json &payload,
    const std::vector<std::string> &headers,
    ResponseCallback on_response)
{
    std::vector<std::string> hdrs = headers;
    bool has_content_type = false;
    for (const auto &h : headers) {
        if (h.rfind("Content-Type:", 0) == 0) {
            has_content_type = true;
            break;
        }
    }

    if (!has_content_type) {
        hdrs.push_back("Content-Type: application/json");
    }

    auto body = payload.dump();
    return sendPostAsync(path, body, hdrs, on_response);
}

std::future<std::pair<std::string, long>> WebServiceClient::sendRequestAsync(
    const std::string &method,
    const std::string &path,
    const std::optional<std::string> &body,
    const std::optional<std::string> &query,
    const std::vector<std::string> &headers,
    ResponseCallback on_response)
{
    auto url = buildUrl(path, query);
    return thread_pool_->enqueue([this, method, url, body, headers, on_response]() {
        const std::string *body_ptr = body ? &(*body) : nullptr;
        auto res = this->performRequest(method, url, body_ptr, headers,
                                       this->default_timeout_ms_, this->max_retries_);
        if (on_response) {
            on_response(res.first, res.second);
        }
        return res;
    });
}

void WebServiceClient::enableResponsePublisher(const std::string &topic_name)
{
  response_pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  publish_responses_ = true;
  RCLCPP_INFO(this->get_logger(), "Enabled response publisher on topic: %s", topic_name.c_str());
}

void WebServiceClient::setAuthHeader(const std::string &auth_header) {
    auth_header_ = auth_header;
}

size_t WebServiceClient::getThreadPoolQueueSize() const {
    return thread_pool_->queue_size();
}

size_t WebServiceClient::getCurlPoolAvailable() const {
    return curl_pool_->available();
}

} // namespace web_service_client