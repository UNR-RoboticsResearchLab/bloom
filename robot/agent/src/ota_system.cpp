#include "ota_system.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>

namespace bloom_agent::ota {

CommandResult run_command(const std::vector<std::string>& args, std::chrono::seconds timeout) {
  CommandResult result;
  if (args.empty()) return result;
  std::vector<char*> argv;
  for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
  argv.push_back(nullptr);

  int out_pipe[2];
  if (pipe2(out_pipe, O_CLOEXEC) != 0) return result;
  const pid_t pid = fork();
  if (pid < 0) {
    ::close(out_pipe[0]);
    ::close(out_pipe[1]);
    return result;
  }
  if (pid == 0) {
    setpgid(0, 0);
    const int devnull = ::open("/dev/null", O_RDONLY);
    dup2(devnull, STDIN_FILENO);
    dup2(out_pipe[1], STDOUT_FILENO);
    dup2(out_pipe[1], STDERR_FILENO);
    close_range(3, ~0U, 0);
    execvp(argv[0], argv.data());
    _exit(127);
  }
  ::close(out_pipe[1]);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (left.count() <= 0) {
      result.timed_out = true;
      break;
    }
    pollfd p{out_pipe[0], POLLIN, 0};
    const int r = poll(&p, 1, static_cast<int>(std::min<long long>(left.count(), 500)));
    if (r < 0 && errno != EINTR) break;
    if (r > 0) {
      char buf[1024];
      const ssize_t n = ::read(out_pipe[0], buf, sizeof(buf));
      if (n <= 0) break;
      if (result.output.size() < 4096) result.output.append(buf, static_cast<std::size_t>(n));
    }
  }
  ::close(out_pipe[0]);

  int status = 0;
  if (result.timed_out) {
    kill(-pid, SIGKILL);
    kill(pid, SIGKILL);
    waitpid(pid, &status, 0);
    return result;
  }
  while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  if (WIFEXITED(status)) result.exit_code = WEXITSTATUS(status);
  return result;
}

namespace {
std::string unit_name(const std::string& job_id) { return "fleet-ota-" + job_id; }

std::string one_line(std::string s) {
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
  const auto nl = s.rfind('\n');
  if (nl != std::string::npos) s = s.substr(nl + 1);
  if (s.size() > 200) s.resize(200);
  return s;
}
}  // namespace

std::vector<std::string> build_launch_argv(const OtaConfig& cfg, const LaunchRequest& r, bool prefix_sudo) {
  std::vector<std::string> argv;
  if (prefix_sudo) argv = {"sudo", "-n"};
  const auto env = [](const char* name, const std::string& value) { return std::string("--setenv=") + name + "=" + value; };
  argv.insert(argv.end(), {
      "systemd-run", "--unit=" + unit_name(r.job_id), "--collect", "--no-block", "--quiet",
      "--description=Bloom fleet OTA " + std::string(to_string(r.action)) + " " + r.job_id,
      env("FLEET_OTA_JOB_ID", r.job_id),
      env("FLEET_OTA_STATE_DIR", cfg.state_dir),
      env("FLEET_OTA_APT_SOURCES", cfg.apt_sources_path),
      env("FLEET_OTA_METAPACKAGE", cfg.metapackage),
      env("FLEET_OTA_ROS_SERVICE", cfg.ros_service),
      env("FLEET_OTA_IDLE_CMD", cfg.idle_command),
      env("FLEET_OTA_HEALTH_CMD", cfg.healthcheck_command),
      env("FLEET_OTA_HEALTH_TIMEOUT", std::to_string(cfg.healthcheck_timeout_seconds)),
      "--", cfg.updater_path, to_string(r.action), r.version});
  return argv;
}

Outcome SystemdOtaSystem::launch(const LaunchRequest& request) {
  const auto res = run_command(build_launch_argv(cfg_, request, use_sudo_), std::chrono::seconds(20));
  if (res.timed_out) return {false, "systemd-run timed out"};
  if (res.exit_code != 0) return {false, "systemd-run failed: " + (res.output.empty() ? "exit " + std::to_string(res.exit_code) : one_line(res.output))};
  return {true, ""};
}

Outcome SystemdOtaSystem::request_cancel(const std::string& job_id) {
  // `cancel` only needs to know where the state dir is; everything else comes from the job.
  std::vector<std::string> argv;
  if (use_sudo_) argv = {"sudo", "-n"};
  argv.insert(argv.end(), {"env", "FLEET_OTA_STATE_DIR=" + cfg_.state_dir, cfg_.updater_path, "cancel", job_id});
  const auto res = run_command(argv, std::chrono::seconds(10));
  if (res.exit_code != 0) return {false, "cancel request failed: " + one_line(res.output)};
  return {true, ""};
}

bool SystemdOtaSystem::unit_active(const std::string& job_id) {
  const auto res = run_command({"systemctl", "is-active", "--quiet", unit_name(job_id) + ".service"}, std::chrono::seconds(10));
  return res.exit_code == 0;
}

std::string SystemdOtaSystem::installed_version() {
  const auto res = run_command({"dpkg-query", "-W", "-f=${Version}", cfg_.metapackage}, std::chrono::seconds(10));
  if (res.exit_code != 0) return "";
  return one_line(res.output);
}

std::optional<JobStatus> SystemdOtaSystem::read_state() {
  std::ifstream in(cfg_.state_dir + "/state.json");
  if (!in) return std::nullopt;
  std::stringstream ss;
  ss << in.rdbuf();
  return parse_state_file(ss.str());
}

}  // namespace bloom_agent::ota
