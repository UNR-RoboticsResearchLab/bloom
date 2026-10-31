// App-level keepalive for the tunnel. Campus networks and proxies silently drop idle or broken
// connections without a FIN, so TCP alone can take many minutes to notice. The agent pings every
// ping_interval and treats the link as dead if nothing at all has arrived for dead_after.
#pragma once

#include <chrono>

namespace bloom_agent {

class Liveness {
 public:
  using clock = std::chrono::steady_clock;

  Liveness(clock::duration ping_interval, clock::duration dead_after, clock::time_point now)
      : ping_interval_(ping_interval), dead_after_(dead_after), last_received_(now), last_ping_(now) {}

  void on_received(clock::time_point now) { last_received_ = now; }
  bool ping_due(clock::time_point now) const { return now - last_ping_ >= ping_interval_; }
  void on_ping_sent(clock::time_point now) { last_ping_ = now; }
  bool is_dead(clock::time_point now) const { return now - last_received_ >= dead_after_; }

 private:
  clock::duration ping_interval_;
  clock::duration dead_after_;
  clock::time_point last_received_;
  clock::time_point last_ping_;
};

}  // namespace bloom_agent
