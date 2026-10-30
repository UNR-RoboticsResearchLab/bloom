#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>

#include "config.h"

using namespace bloom_agent;

namespace {
AgentConfig load(const std::string& json) {
  char tmpl[] = "/tmp/ota_cfg_XXXXXX";
  const std::string dir = mkdtemp(tmpl);
  const std::string path = dir + "/agent.json";
  std::ofstream(path) << json;
  struct Cleanup { std::string d; ~Cleanup() { (void)!system(("rm -rf " + d).c_str()); } } cleanup{dir};
  return load_config(path, /*require_credentials=*/false);
}
}  // namespace

TEST(OtaConfig, IsOffByDefaultWithSafeDefaults) {
  const auto cfg = load(R"({"backend_url":"https://b.example"})");
  EXPECT_FALSE(cfg.ota.enabled);
  EXPECT_TRUE(cfg.ota.allowed_suites.empty());
  EXPECT_EQ(cfg.ota.metapackage, "bloom-robot-core");
  EXPECT_EQ(cfg.ota.healthcheck_timeout_seconds, 120);
}

TEST(OtaConfig, ReadsEverySetting) {
  const auto cfg = load(R"({"backend_url":"https://b.example","ota":{
    "enabled":true,"apt_sources_path":"/a.list","metapackage":"meta","allowed_suites":["x","y"],
    "signing_key_path":"/k.pub","healthcheck_timeout_seconds":30,"idle_command":"/idle",
    "healthcheck_command":"/health","ros_service":"ros.service","updater_path":"/u","state_dir":"/s","job_store_path":"/j"}})");
  EXPECT_TRUE(cfg.ota.enabled);
  EXPECT_EQ(cfg.ota.apt_sources_path, "/a.list");
  EXPECT_EQ(cfg.ota.metapackage, "meta");
  EXPECT_EQ(cfg.ota.allowed_suites, (std::vector<std::string>{"x", "y"}));
  EXPECT_EQ(cfg.ota.signing_key_path, "/k.pub");
  EXPECT_EQ(cfg.ota.healthcheck_timeout_seconds, 30);
  EXPECT_EQ(cfg.ota.idle_command, "/idle");
  EXPECT_EQ(cfg.ota.healthcheck_command, "/health");
  EXPECT_EQ(cfg.ota.ros_service, "ros.service");
  EXPECT_EQ(cfg.ota.updater_path, "/u");
  EXPECT_EQ(cfg.ota.state_dir, "/s");
  EXPECT_EQ(cfg.ota.job_store_path, "/j");
}

TEST(OtaConfig, RejectsAnAbsurdHealthcheckTimeoutWhenEnabled) {
  EXPECT_THROW(load(R"({"backend_url":"https://b.example","ota":{"enabled":true,"healthcheck_timeout_seconds":0}})"), std::runtime_error);
  EXPECT_NO_THROW(load(R"({"backend_url":"https://b.example","ota":{"enabled":false,"healthcheck_timeout_seconds":0}})"));
}
