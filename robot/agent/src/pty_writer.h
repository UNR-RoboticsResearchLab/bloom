// Non-blocking input to a PTY. Keystrokes used to be written with a blocking loop, which wedges the
// whole (single-threaded) agent on a large paste: the shell echoes input back into a buffer nobody is
// draining because the loop is busy writing, so the tty stops accepting input and the rest of the paste
// is lost. Here the write is queued and continued whenever the master becomes writable, so the event
// loop keeps draining the shell's output meanwhile.
#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <memory>
#include <string>
#include <string_view>

namespace bloom_agent {

class PtyWriter : public std::enable_shared_from_this<PtyWriter> {
 public:
  // More than this waiting to be written means the shell isn't consuming input at all; further input
  // is dropped rather than growing without bound.
  static constexpr std::size_t kMaxPending = 4 << 20;

  // Duplicates `fd`; the caller keeps ownership of the original.
  static std::shared_ptr<PtyWriter> create(boost::asio::io_context& io, int fd) {
    return std::shared_ptr<PtyWriter>(new PtyWriter(io, fd));
  }

  // Queues `data` and writes as much as the kernel takes now. False if it was dropped.
  bool write(std::string_view data);
  std::size_t pending() const { return buffer_.size() - offset_; }
  // Stops any pending wait. Call before the owner goes away.
  void cancel();

 private:
  PtyWriter(boost::asio::io_context& io, int fd);
  void pump();

  boost::asio::posix::stream_descriptor descriptor_;
  std::string buffer_;
  std::size_t offset_ = 0;
  bool waiting_ = false;
  bool cancelled_ = false;
};

}  // namespace bloom_agent
