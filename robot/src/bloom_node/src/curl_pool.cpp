#include "bloom_node/curl_pool.h"

using namespace bloom_node;


CurlPool::CurlPool(size_t pool_size) : max_pool_size_(pool_size) {
  // Pre-allocate handles
  for (size_t i = 0; i < pool_size; ++i) {
    available_handles_.push(create_handle());
  }
}

CurlPool::~CurlPool() {
  shutdown_ = true;
  std::lock_guard<std::mutex> lock(pool_mutex_);
  while (!available_handles_.empty()) {
    CURL* handle = available_handles_.front();
    available_handles_.pop();
    destroy_handle(handle);
  }
}

CURL* CurlPool::acquire() {
  std::lock_guard<std::mutex> lock(pool_mutex_);

  if (!available_handles_.empty()) {
    CURL* handle = available_handles_.front();
    available_handles_.pop();
    return handle;
  }

  // Create new handle if pool is depleted
  return create_handle();
}

void CurlPool::release(CURL* handle) {
  if (!handle) return;

  // Reset the handle to a clean state
  curl_easy_reset(handle);

  std::lock_guard<std::mutex> lock(pool_mutex_);

  if (available_handles_.size() < max_pool_size_) {
    available_handles_.push(handle);
  } else {
    destroy_handle(handle);
  }
}

CURL* CurlPool::create_handle() {
  CURL* curl = curl_easy_init();
  if (!curl) {
    throw std::runtime_error("Failed to initialize CURL handle");
  }
  return curl;
}

void CurlPool::destroy_handle(CURL* handle) {
  if (handle) {
    curl_easy_cleanup(handle);
  }
}
