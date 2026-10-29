#include "ota_system.h"

#include <gtest/gtest.h>

#include <algorithm>

using namespace bloom_agent;
using namespace bloom_agent::ota;

TEST(OtaLaunch, RunsTheUpdaterInItsOwnTransientUnit) {
  OtaConfig cfg;
  const auto argv = build_launch_argv(cfg, {"job-7", Action::Apply, "1.4.2-1jazzy"}, /*prefix_sudo=*/true);
  EXPECT_EQ(argv[0], "sudo");
  EXPECT_EQ(argv[1], "-n");
  EXPECT_EQ(argv[2], "systemd-run");
  EXPECT_NE(std::find(argv.begin(), argv.end(), "--unit=fleet-ota-job-7"), argv.end());
  EXPECT_NE(std::find(argv.begin(), argv.end(), "--collect"), argv.end());
  EXPECT_NE(std::find(argv.begin(), argv.end(), "--no-block"), argv.end());
  // The updater and its two arguments come last, after "--".
  const auto n = argv.size();
  EXPECT_EQ(argv[n - 4], "--");
  EXPECT_EQ(argv[n - 3], cfg.updater_path);
  EXPECT_EQ(argv[n - 2], "apply");
  EXPECT_EQ(argv[n - 1], "1.4.2-1jazzy");
}

TEST(OtaLaunch, NoSudoWhenRoot) {
  const auto argv = build_launch_argv(OtaConfig{}, {"j", Action::Stage, "1.0"}, false);
  EXPECT_EQ(argv[0], "systemd-run");
}

TEST(OtaLaunch, EverythingIsOneArgumentEachAndNoShellIsInvolved) {
  // Even if validation were bypassed, a hostile string stays inside one argv element.
  const auto argv = build_launch_argv(OtaConfig{}, {"a b", Action::Stage, "1.0; reboot"}, true);
  EXPECT_NE(std::find(argv.begin(), argv.end(), "1.0; reboot"), argv.end());
  EXPECT_EQ(std::find(argv.begin(), argv.end(), "sh"), argv.end());
  EXPECT_EQ(std::find(argv.begin(), argv.end(), "bash"), argv.end());
  EXPECT_EQ(std::find(argv.begin(), argv.end(), "-c"), argv.end());
}

TEST(OtaLaunch, ConfigReachesTheUpdaterThroughTheEnvironment) {
  OtaConfig cfg;
  cfg.metapackage = "my-meta";
  cfg.healthcheck_timeout_seconds = 45;
  cfg.ros_service = "ros.service";
  const auto argv = build_launch_argv(cfg, {"j", Action::Stage, "1.0"}, false);
  auto has = [&](const std::string& a) { return std::find(argv.begin(), argv.end(), a) != argv.end(); };
  EXPECT_TRUE(has("--setenv=FLEET_OTA_METAPACKAGE=my-meta"));
  EXPECT_TRUE(has("--setenv=FLEET_OTA_HEALTH_TIMEOUT=45"));
  EXPECT_TRUE(has("--setenv=FLEET_OTA_ROS_SERVICE=ros.service"));
  EXPECT_TRUE(has("--setenv=FLEET_OTA_JOB_ID=j"));
  // Test-only overrides (stubbed apt/dpkg) must never be settable from here.
  EXPECT_TRUE(std::none_of(argv.begin(), argv.end(), [](const std::string& a) { return a.find("FLEET_OTA_APT_GET") != std::string::npos; }));
}

TEST(OtaRunCommand, CapturesOutputAndExitCode) {
  const auto ok = run_command({"sh", "-c", "echo hi; exit 0"}, std::chrono::seconds(5));
  EXPECT_EQ(ok.exit_code, 0);
  EXPECT_EQ(ok.output, "hi\n");
  EXPECT_EQ(run_command({"sh", "-c", "exit 3"}, std::chrono::seconds(5)).exit_code, 3);
  EXPECT_NE(run_command({"/nonexistent/binary"}, std::chrono::seconds(5)).exit_code, 0);
}

TEST(OtaRunCommand, KillsACommandThatHangs) {
  const auto res = run_command({"sleep", "30"}, std::chrono::seconds(1));
  EXPECT_TRUE(res.timed_out);
}
