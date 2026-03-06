#ifndef BLOOM_NODE_CURL_POOL_H
#define BLOOM_NODE_CURL_POOL_H

#include <curl/curl.h>
#include <queue>
#include <memory>
#include <mutex>
#include <atomic>
#include <stdexcept>

namespace bloom_node {

/**
 * Thread-safe CURL handle pool to enable connection reuse and improve performance.
 * Prevents recreating CURL handles for every request.
 */
class CurlPool {
public:
  CurlPool(size_t pool_size = 4);
  ~CurlPool();

  /// Get a CURL handle from the pool, creating one if needed
  CURL* acquire();

  /// Return a handle to the pool for reuse
  void release(CURL* handle);

  /// Get current pool size
  size_t available() const {
    std::lock_guard<std::mutex> lock(pool_mutex_);
    return available_handles_.size();
  }

private:
  std::queue<CURL*> available_handles_;
  size_t max_pool_size_;
  mutable std::mutex pool_mutex_;
  std::atomic<bool> shutdown_{false};

  CURL* create_handle();
  void destroy_handle(CURL* handle);
};

} // namespace bloom_node

#endif // BLOOM_NODE_CURL_POOL_H
