// Reconnect delay: "full jitter" exponential backoff. Attempt n waits a uniformly random time in
// [floor, min(cap, base * 2^n)]. The jitter keeps a fleet that lost the backend at the same moment
// (backend deploy, campus Wi-Fi blip) from reconnecting in lockstep. Never gives up.
#pragma once

#include <algorithm>
#include <chrono>
#include <functional>

namespace bloom_agent {

class Backoff {
 public:
  using ms = std::chrono::milliseconds;
  // Returns a uniform double in [0, 1). Injected so tests are deterministic.
  using Random = std::function<double()>;

  Backoff(ms base, ms cap, Random random, ms floor = ms(250))
      : base_(base), cap_(cap), floor_(floor), random_(std::move(random)) {}

  ms next() {
    const ms ceiling = this->ceiling();
    if (attempt_ < 30) ++attempt_;  // 2^30 * base is far past any sane cap
    const auto jittered = ms(static_cast<ms::rep>(random_() * static_cast<double>(ceiling.count())));
    return std::clamp(jittered, std::min(floor_, ceiling), ceiling);
  }

  // Upper bound for the next delay.
  ms ceiling() const {
    const auto grown = base_.count() * (ms::rep(1) << attempt_);
    return ms(std::min<ms::rep>(grown, cap_.count()));
  }

  void reset() { attempt_ = 0; }
  int attempts() const { return attempt_; }

 private:
  ms base_;
  ms cap_;
  ms floor_;
  Random random_;
  int attempt_ = 0;
};

}  // namespace bloom_agent
