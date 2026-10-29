#include "pty_writer.h"

#include <unistd.h>

#include <cerrno>

#include "log.h"

namespace bloom_agent {

PtyWriter::PtyWriter(boost::asio::io_context& io, int fd) : descriptor_(io, ::dup(fd)) {}

bool PtyWriter::write(std::string_view data) {
  if (cancelled_) return false;
  if (pending() + data.size() > kMaxPending) {
    log::warn("dropping terminal input: the shell isn't reading it");
    return false;
  }
  buffer_.append(data);
  pump();
  return true;
}

void PtyWriter::cancel() {
  cancelled_ = true;
  boost::system::error_code ignored;
  descriptor_.close(ignored);  // completes a pending wait with operation_aborted
}

void PtyWriter::pump() {
  while (offset_ < buffer_.size() && !cancelled_) {
    const ssize_t n = ::write(descriptor_.native_handle(), buffer_.data() + offset_, buffer_.size() - offset_);
    if (n > 0) {
      offset_ += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && errno == EAGAIN) {
      if (!waiting_) {
        waiting_ = true;
        descriptor_.async_wait(boost::asio::posix::stream_descriptor::wait_write,
                               [self = shared_from_this()](const boost::system::error_code& ec) {
                                 self->waiting_ = false;
                                 if (!ec) self->pump();
                               });
      }
      return;
    }
    // EIO: the shell is gone. The output reader reports that; nothing more to deliver.
    buffer_.clear();
    offset_ = 0;
    return;
  }
  buffer_.clear();
  offset_ = 0;
}

}  // namespace bloom_agent
