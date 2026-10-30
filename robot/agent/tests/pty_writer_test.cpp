// A paste bigger than the PTY's buffers must go through while the shell's echo is being drained by the
// same event loop (what the agent does), not wedge it.
#include "pty_writer.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>

#include "pty_process.h"

using namespace bloom_agent;
namespace asio = boost::asio;

namespace {
// Runs the loop, draining the PTY's output into `out` like the agent's reader does, until `done`
// says so or the deadline passes.
struct Drain {
  Drain(asio::io_context& io, int fd) : descriptor(io, ::dup(fd)) { read(); }
  void read() {
    descriptor.async_read_some(asio::buffer(buf), [this](const boost::system::error_code& ec, std::size_t n) {
      if (ec) return;
      out.append(buf.data(), n);
      read();
    });
  }
  asio::posix::stream_descriptor descriptor;
  std::array<char, 4096> buf{};
  std::string out;
};

bool run_until(asio::io_context& io, const std::function<bool()>& done, std::chrono::seconds limit) {
  asio::steady_timer poll(io);
  const auto deadline = std::chrono::steady_clock::now() + limit;
  std::function<void()> tick = [&] {
    if (done() || std::chrono::steady_clock::now() > deadline) {
      io.stop();
      return;
    }
    poll.expires_after(std::chrono::milliseconds(10));
    poll.async_wait([&](const boost::system::error_code&) { tick(); });
  };
  tick();
  io.run();
  return done();
}
}  // namespace

TEST(PtyWriter, ALargePasteIsDeliveredWhileOutputIsDrained) {
  asio::io_context io;
  auto pty = PosixPty::spawn("/bin/sh", 100, 30);
  Drain drain(io, pty->fd());
  auto writer = PtyWriter::create(io, pty->fd());

  std::string big;
  for (int i = 0; i < 1536; ++i) big += std::string(79, 'x') + "\n";  // 120 KB in terminal-sized lines
  ASSERT_EQ(big.size(), 122880u);
  ASSERT_TRUE(writer->write("wc -c <<'EOF'\n" + big + "EOF\necho FIN$((1+1))\n"));

  ASSERT_TRUE(run_until(io, [&] { return drain.out.find("FIN2") != std::string::npos; }, std::chrono::seconds(20)));
  EXPECT_NE(drain.out.find("122880"), std::string::npos) << "the shell received a truncated paste";
  EXPECT_EQ(writer->pending(), 0u);
  writer->cancel();
}

TEST(PtyWriter, SmallInputIsWrittenImmediately) {
  asio::io_context io;
  auto pty = PosixPty::spawn("/bin/sh", 80, 24);
  Drain drain(io, pty->fd());
  auto writer = PtyWriter::create(io, pty->fd());

  ASSERT_TRUE(writer->write("echo HEL$((1+1))O\n"));
  EXPECT_EQ(writer->pending(), 0u);
  ASSERT_TRUE(run_until(io, [&] { return drain.out.find("HEL2O") != std::string::npos; }, std::chrono::seconds(10)));
  writer->cancel();
}

TEST(PtyWriter, StopsAcceptingInputThatTheShellNeverReads) {
  asio::io_context io;
  auto pty = PosixPty::spawn("/bin/sh", 80, 24);
  auto writer = PtyWriter::create(io, pty->fd());

  // Nothing drains the output here, so the shell's echo backs up and it stops reading.
  const std::string chunk(1 << 20, 'a');
  bool refused = false;
  for (int i = 0; i < 12 && !refused; ++i) refused = !writer->write(chunk + "\n");
  EXPECT_TRUE(refused);
  EXPECT_LE(writer->pending(), PtyWriter::kMaxPending);
  writer->cancel();
}
