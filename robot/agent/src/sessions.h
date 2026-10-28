// The shell sessions open on this robot, keyed by the backend's session id.
#pragma once

#include <map>
#include <memory>

#include "protocol.h"
#include "pty_process.h"

namespace bloom_agent {

class SessionManager {
 public:
  // The backend enforces one session per robot; this is a backstop against a misbehaving relay.
  static constexpr std::size_t kMaxSessions = 4;

  explicit SessionManager(PtyFactory factory) : factory_(std::move(factory)) {}
  ~SessionManager() { close_all(); }

  // nullptr if the id is already open, the limit is reached, or the shell couldn't start.
  IPty* open(const Uuid& id, int cols, int rows);
  IPty* find(const Uuid& id);
  bool resize(const Uuid& id, int cols, int rows);
  // Terminates the shell. False if there was no such session.
  bool close(const Uuid& id);
  // Removes a session whose shell already exited, returning it so the caller can collect the
  // exit code.
  std::unique_ptr<IPty> release(const Uuid& id);
  // Called whenever the tunnel drops: nobody can see these shells any more.
  void close_all();
  std::size_t size() const { return sessions_.size(); }

 private:
  PtyFactory factory_;
  std::map<Uuid, std::unique_ptr<IPty>> sessions_;
};

}  // namespace bloom_agent
