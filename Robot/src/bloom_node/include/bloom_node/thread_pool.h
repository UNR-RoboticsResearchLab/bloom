#ifndef BLOOM_NODE_THREAD_POOL_H
#define BLOOM_NODE_THREAD_POOL_H

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <functional>
#include <atomic>
#include <vector>
#include <future>

namespace bloom_node {

/**
 * Thread-safe, bounded thread pool for async HTTP operations.
 * Prevents unlimited thread spawning from std::async.
 */
class ThreadPool {
public:
  explicit ThreadPool(size_t num_threads = 4);
  ~ThreadPool();

  /// Enqueue a task and return a future to the result
  template<typename F, typename... Args>
  auto enqueue(F&& f, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<return_type> res = task->get_future();
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if (!running_) {
        throw std::runtime_error("Cannot enqueue into stopped ThreadPool");
      }
      tasks_.emplace([task]() { (*task)(); });
    }
    condition_.notify_one();
    return res;
  }

  /// Wait for all queued tasks to complete
  void wait_all();

  /// Get current queue size (for monitoring)
  size_t queue_size() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return tasks_.size();
  }

private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;

  mutable std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> running_{true};

  void worker_thread();
};

} // namespace bloom_node

#endif // BLOOM_NODE_THREAD_POOL_H
