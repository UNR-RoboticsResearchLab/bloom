#include "sessions.h"

#include "log.h"

namespace bloom_agent {

IPty* SessionManager::open(const Uuid& id, int cols, int rows) {
  if (sessions_.count(id) || sessions_.size() >= kMaxSessions) return nullptr;
  std::unique_ptr<IPty> pty;
  try {
    pty = factory_(cols, rows);
  } catch (const std::exception& e) {
    log::error(std::string("failed to start shell: ") + e.what());
    return nullptr;
  }
  if (!pty) return nullptr;
  auto* raw = pty.get();
  sessions_.emplace(id, std::move(pty));
  return raw;
}

IPty* SessionManager::find(const Uuid& id) {
  auto it = sessions_.find(id);
  return it == sessions_.end() ? nullptr : it->second.get();
}

bool SessionManager::resize(const Uuid& id, int cols, int rows) {
  auto* pty = find(id);
  if (!pty) return false;
  pty->resize(cols, rows);
  return true;
}

bool SessionManager::close(const Uuid& id) {
  auto it = sessions_.find(id);
  if (it == sessions_.end()) return false;
  it->second->terminate();
  sessions_.erase(it);
  return true;
}

std::unique_ptr<IPty> SessionManager::release(const Uuid& id) {
  auto it = sessions_.find(id);
  if (it == sessions_.end()) return nullptr;
  auto pty = std::move(it->second);
  sessions_.erase(it);
  return pty;
}

void SessionManager::close_all() {
  for (auto& [id, pty] : sessions_) pty->terminate();
  sessions_.clear();
}

}  // namespace bloom_agent
