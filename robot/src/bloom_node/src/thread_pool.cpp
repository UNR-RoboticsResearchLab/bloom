#include "bloom_node/thread_pool.h"

namespace bloom_node {

ThreadPool::ThreadPool(size_t num_threads) {
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this] { worker_thread(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    running_ = false;
  }
  condition_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void ThreadPool::worker_thread() {
  while (running_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    condition_.wait(lock, [this] { return !tasks_.empty() || !running_; });

    if (!running_ && tasks_.empty()) {
      break;
    }

    if (!tasks_.empty()) {
      auto task = std::move(tasks_.front());
      tasks_.pop();
      lock.unlock();
      task();
    }
  }
}

void ThreadPool::wait_all() {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  condition_.wait(lock, [this] { return tasks_.empty(); });
}

} // namespace bloom_node
