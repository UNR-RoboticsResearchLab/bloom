#include "bloom_node/web_service_client.h"

namespace web_service_client{

WebServiceClient::WebServiceClient(
    const std::string &node_name,
    const std::string &base_url,
    int default_timeout_ms,
    int max_retries
) : Node(node_name),
  base_url_(base_url),
  default_timeout_ms_(default_timeout_ms),
  max_retries_(max_retries)
{
     // Initialize libcurl globally (thread-safe with mutex)
    {
        std::lock_guard<std::mutex> lk(curl_init_mutex_);
        curl_global_init(CURL_GLOBAL_ALL);
    }

    // Optional: create a Trigger service that will perform a simple GET to "/" path
    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "web_service_trigger",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            (void)req;
            // fire off async GET to base path; publish result via publisher if enabled
            this->sendGetAsync("/", std::nullopt, {}, [this, res](const std::string &body, long code) mutable {
            res->success = (code >= 200 && code < 300);
            res->message = "HTTP code: " + std::to_string(code) + ", body length: " + std::to_string(body.size());
        });
      });

    RCLCPP_INFO(this->get_logger(), "WebServiceClient node constructed for base_url=%s", base_url_.c_str());

}


WebServiceClient::~WebServiceClient()
{
  running_.store(false);
  std::lock_guard<std::mutex> lk(curl_init_mutex_);
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

inline std::pair<std::string, long> WebServiceClient::performRequest(
  const std::string &method,
  const std::string &url,
  const std::string *body,
  const std::vector<std::string> &headers,
  int timeout_ms,
  int retries)
{

    CURL *curl = curl_easy_init();
    if (!curl)
    {
        RCLCPP_ERROR(this->get_logger(), "curl_easy_init() failed");
        return {"", 0};
    }

    std::string response_data;

    struct curl_slist *hdrs = nullptr;
    
    //merge current headers w / auth

    for (const auto &h : headers)
    {
        hdrs = curl_slist_append(hdrs, h.c_str());
    }

    if (!auth_header_.empty())
    {
        hdrs = curl_slist_append(hdrs, auth_header_.c_str());
    }

    // set headers / options
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &WebServiceClient::writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    if (hdrs) curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);


    // add body
    if (method == "POST")
    {
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        if (body)
        {
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body->size());
        }
    }

    long http_code = 0;
    CURLcode response;
    int attempt = 0;

    // attempt requests
    do
    {
        response = curl_easy_perform(curl);

        if (response == CURLE_OK)
        {
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
            break;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "curl_easy_perform() failed: %s (attempt %d/%d)",
            curl_easy_strerror(response), attempt + 1, retries + 1);
            std::this_thread::sleep_for(200ms * (attempt + 1));
        }
        attempt++;
    } while (attempt <= retries && running_.load());
    
    if (hdrs) curl_slist_free_all(hdrs);
    curl_easy_cleanup(curl);

    // Publish if enabled
    if (publish_responses_ && response_pub_)
    {
        auto msg = std_msgs::msg::String();
        msg.data = response_data;
        response_pub_->publish(msg);
    }

    return {response_data, http_code};
}


std::future<std::pair<std::string, long>> WebServiceClient::sendGetAsync(
  const std::string &path,
  const std::optional<std::string> &query,
  const std::vector<std::string> &headers,
  ResponseCallback on_response)
{
    auto url = buildUrl(path, query);
    return std::async(std::launch::async, [this, url, headers, on_response]() {

        auto res = this->performRequest("GET", url, nullptr, headers, this->default_timeout_ms_, this->max_retries_);
        
        if (on_response) on_response(res.first, res.second);
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
    return std::async(std::launch::async, [this, url, body, headers, on_response]() {

        auto res = this->performRequest("POST", url, &body, headers, this->default_timeout_ms_, this->max_retries_);

        if (on_response) on_response(res.first, res.second);
    return res;
    });
}

inline std::future<std::pair<std::string, long>> WebServiceClient::sendJsonPostAsync(
    const std::string &path,
    const json &payload,
    const std::vector<std::string> &headers,
    ResponseCallback on_response)
{
    std::vector<std::string> hdrs = headers;
    bool has_content_type = false;
    for (const auto &h : headers) if (h.rfind("Content-Type:", 0) == 0) has_content_type = true;

    if (!has_content_type) hdrs.push_back("Content-Type: application/json");

    auto body = payload.dump();
    return sendPostAsync(path, body, hdrs, on_response);
}

void WebServiceClient::enableResponsePublisher(const std::string &topic_name)
{
  response_pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  publish_responses_ = true;
  RCLCPP_INFO(this->get_logger(), "Enabled response publisher on topic: %s", topic_name.c_str());
}

void WebServiceClient::setAuthHeader(const std::string &auth_header)
{
  // User should supply a properly formatted header, e.g. "Authorization: Bearer <token>"
  auth_header_ = auth_header;
}




} // namespace web_service_client