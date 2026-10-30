// The OTA manager against a fake robot (no systemd, apt or files except the job store): manifest
// handling, idempotency, state forwarding, cancel rules, and what is re-sent after a reconnect.
#include "ota_manager.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>

#include "ota_test_util.h"

using namespace bloom_agent;
using namespace bloom_agent::ota;
using ota_test::signed_manifest;
using ota_test::TestKey;

namespace {

struct FakeSystem : IOtaSystem {
  Outcome launch(const LaunchRequest& r) override {
    if (throws) throw std::runtime_error("boom");
    launches.push_back(r);
    return launch_result;
  }
  Outcome request_cancel(const std::string& id) override {
    cancels.push_back(id);
    return {true, ""};
  }
  bool unit_active(const std::string&) override { return unit_is_active; }
  std::string installed_version() override { return installed; }
  std::optional<JobStatus> read_state() override { return state_file; }

  std::vector<LaunchRequest> launches;
  std::vector<std::string> cancels;
  Outcome launch_result{true, ""};
  bool unit_is_active = true;
  bool throws = false;
  std::string installed = "1.0.0";
  std::optional<JobStatus> state_file;
};

struct FakeSink : IOtaSink {
  void progress(const JobStatus& s) override { reports.push_back(s); }
  void installed_version(const std::string& v) override { versions.push_back(v); }
  std::vector<JobStatus> reports;
  std::vector<std::string> versions;
  std::vector<JobState> states() const {
    std::vector<JobState> out;
    for (const auto& r : reports) out.push_back(r.state);
    return out;
  }
};

const TestKey& key() {
  static TestKey k;
  return k;
}

class OtaManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char tmpl[] = "/tmp/ota_mgr_XXXXXX";
    dir_ = mkdtemp(tmpl);
    store_path_ = dir_ + "/jobs.json";
    cfg_.allowed_suites = {"jazzy-stable"};
    rebuild();
  }
  void TearDown() override { (void)!system(("rm -rf " + dir_).c_str()); }

  // A fresh manager over the same store file: what an agent restart looks like.
  void rebuild() {
    manager_.reset();
    store_ = std::make_unique<JobStore>(store_path_);
    store_->load();
    manager_ = std::make_unique<OtaManager>(cfg_, sys_, *store_, [] { return key().public_pem; },
                                            [this] { return std::chrono::system_clock::from_time_t(now_); });
    manager_->attach(&sink_);
  }

  void start(const nlohmann::json& manifest) { manager_->on_start_ota(nlohmann::json::array({manifest})); }
  void cancel(const std::string& id) { manager_->on_cancel_ota(nlohmann::json::array({id})); }
  void file(const std::string& id, JobState s, int pct = 0, const std::string& detail = "") { sys_.state_file = JobStatus{id, s, pct, detail}; }
  JobState state_of(const std::string& id) { return store_->find(id)->status.state; }

  OtaConfig cfg_;
  FakeSystem sys_;
  FakeSink sink_;
  std::string dir_, store_path_;
  std::int64_t now_ = 1'800'000'000;
  std::unique_ptr<JobStore> store_;
  std::unique_ptr<OtaManager> manager_;
};

}  // namespace

// ── StartOta ─────────────────────────────────────────────────────────────

TEST_F(OtaManagerTest, AValidManifestIsLaunchedWithArgumentsFromTheManifest) {
  start(signed_manifest(key(), "job-1", "apply", "2.0.0-1jazzy"));
  ASSERT_EQ(sys_.launches.size(), 1u);
  EXPECT_EQ(sys_.launches[0].job_id, "job-1");
  EXPECT_EQ(sys_.launches[0].action, Action::Apply);
  EXPECT_EQ(sys_.launches[0].version, "2.0.0-1jazzy");
  EXPECT_EQ(sink_.states(), std::vector<JobState>{JobState::Queued});
}

TEST_F(OtaManagerTest, InvalidManifestsAreRejectedAndNeverLaunched) {
  auto expired = signed_manifest(key(), "j-expired", "stage", "1.0.0", "jazzy-stable", "2020-01-01T00:00:00Z");
  auto bad_sig = signed_manifest(key(), "j-sig", "stage");
  bad_sig["version"] = "9.9.9";
  auto bad_version = signed_manifest(key(), "j-ver", "stage", "1.0; reboot");
  auto bad_suite = signed_manifest(key(), "j-suite", "stage", "1.0.0", "other-suite");
  for (const auto& m : {expired, bad_sig, bad_version, bad_suite}) start(m);

  EXPECT_TRUE(sys_.launches.empty());
  ASSERT_EQ(sink_.reports.size(), 4u);
  for (const auto& r : sink_.reports) {
    EXPECT_EQ(r.state, JobState::Rejected);
    EXPECT_FALSE(r.detail.empty());
  }
  EXPECT_EQ(sink_.reports[0].detail, "manifest has expired");
  EXPECT_EQ(sink_.reports[1].detail, "manifest signature is invalid");
  EXPECT_EQ(sink_.reports[3].detail, "suite is not allowed on this robot");
}

TEST_F(OtaManagerTest, AManifestWithoutAUsableJobIdIsIgnoredSilently) {
  auto m = signed_manifest(key(), "j", "stage");
  m["jobId"] = "bad id!";
  start(m);
  manager_->on_start_ota(nlohmann::json::array());
  manager_->on_start_ota(nlohmann::json::array({42}));
  EXPECT_TRUE(sink_.reports.empty());
  EXPECT_TRUE(sys_.launches.empty());
}

TEST_F(OtaManagerTest, AManifestMayArriveAsAJsonString) {
  manager_->on_start_ota(nlohmann::json::array({signed_manifest(key(), "job-s", "stage").dump()}));
  EXPECT_EQ(sys_.launches.size(), 1u);
}

TEST_F(OtaManagerTest, ALaunchFailureFailsTheJob) {
  sys_.launch_result = {false, "systemd-run failed"};
  start(signed_manifest(key(), "job-1", "stage"));
  EXPECT_EQ(sink_.states(), (std::vector<JobState>{JobState::Queued, JobState::Failed}));
  EXPECT_EQ(sink_.reports.back().detail, "systemd-run failed");
}

TEST_F(OtaManagerTest, OnlyOneJobRunsAtATime) {
  start(signed_manifest(key(), "job-1", "stage"));
  start(signed_manifest(key(), "job-2", "apply"));
  EXPECT_EQ(sys_.launches.size(), 1u);
  EXPECT_EQ(state_of("job-2"), JobState::Rejected);
  EXPECT_NE(store_->find("job-2")->status.detail.find("job-1"), std::string::npos);
}

// ── Idempotency ──────────────────────────────────────────────────────────

TEST_F(OtaManagerTest, ADuplicateJobIdIsIgnored) {
  const auto m = signed_manifest(key(), "job-1", "stage");
  start(m);
  const auto reports = sink_.reports.size();
  start(m);
  start(m);
  EXPECT_EQ(sys_.launches.size(), 1u);
  EXPECT_EQ(sink_.reports.size(), reports);
}

TEST_F(OtaManagerTest, IdempotencySurvivesAnAgentRestartEvenAfterTheJobFinished) {
  const auto m = signed_manifest(key(), "job-1", "stage");
  start(m);
  file("job-1", JobState::Staged, 100, "done");
  manager_->poll();

  rebuild();
  start(m);  // the backend re-sends after the reconnect
  EXPECT_EQ(sys_.launches.size(), 1u);
}

TEST_F(OtaManagerTest, IdempotencySurvivesARestartWhileTheJobIsStillRunning) {
  const auto m = signed_manifest(key(), "job-1", "apply");
  start(m);
  rebuild();
  start(m);
  EXPECT_EQ(sys_.launches.size(), 1u);
}

TEST_F(OtaManagerTest, ARejectedJobIdIsAlsoRememberedAcrossRestarts) {
  auto bad = signed_manifest(key(), "job-x", "stage", "1.0.0", "nope");
  start(bad);
  rebuild();
  const auto before = sink_.reports.size();
  start(bad);
  EXPECT_EQ(sink_.reports.size(), before);
}

// ── State transitions ────────────────────────────────────────────────────

TEST_F(OtaManagerTest, StateFileChangesAreForwardedInOrderWithoutDuplicates) {
  start(signed_manifest(key(), "job-1", "apply"));
  file("job-1", JobState::Applying, 20, "installing");
  manager_->poll();
  manager_->poll();  // unchanged: nothing to forward
  file("job-1", JobState::Applying, 60, "restarting");
  manager_->poll();
  file("job-1", JobState::Healthcheck, 80, "checking");
  manager_->poll();
  file("job-1", JobState::Success, 100, "healthy");
  manager_->poll();
  manager_->poll();

  EXPECT_EQ(sink_.states(), (std::vector<JobState>{JobState::Queued, JobState::Applying, JobState::Applying, JobState::Healthcheck, JobState::Success}));
  EXPECT_EQ(sink_.reports[2].percent, 60);
  EXPECT_EQ(sink_.reports.back().detail, "healthy");
  EXPECT_EQ(state_of("job-1"), JobState::Success);
}

TEST_F(OtaManagerTest, AFinishedApplyReportsTheNewInstalledVersion) {
  start(signed_manifest(key(), "job-1", "apply", "2.0.0"));
  sys_.installed = "2.0.0";
  file("job-1", JobState::Success, 100);
  manager_->poll();
  EXPECT_EQ(sink_.versions, std::vector<std::string>{"2.0.0"});
}

TEST_F(OtaManagerTest, ARolledBackApplyReportsTheRestoredVersion) {
  start(signed_manifest(key(), "job-1", "apply", "2.0.0"));
  file("job-1", JobState::RolledBack, 100, "unhealthy");
  manager_->poll();
  EXPECT_EQ(sink_.states().back(), JobState::RolledBack);
  EXPECT_EQ(sink_.versions, std::vector<std::string>{"1.0.0"});
}

TEST_F(OtaManagerTest, AStateFileOfAnotherJobIsIgnored) {
  start(signed_manifest(key(), "job-2", "stage"));
  file("job-1", JobState::Success, 100);  // left over from an earlier job
  manager_->poll();
  EXPECT_EQ(state_of("job-2"), JobState::Queued);
}

TEST_F(OtaManagerTest, AVanishedUpdaterFailsTheJobOnlyAfterTheStartGrace) {
  start(signed_manifest(key(), "job-1", "apply"));
  sys_.unit_is_active = false;
  now_ += 5;
  manager_->poll();
  EXPECT_EQ(state_of("job-1"), JobState::Queued);
  now_ += 30;
  manager_->poll();
  EXPECT_EQ(state_of("job-1"), JobState::Failed);
}

TEST_F(OtaManagerTest, AnUpdaterThatFinishedJustBeforeVanishingIsNotCalledFailed) {
  start(signed_manifest(key(), "job-1", "stage"));
  sys_.unit_is_active = false;
  now_ += 60;
  file("job-1", JobState::Staged, 100, "ok");
  manager_->poll();
  EXPECT_EQ(state_of("job-1"), JobState::Staged);
}

// ── CancelOta ────────────────────────────────────────────────────────────

TEST_F(OtaManagerTest, CancelWorksWhileStaging) {
  start(signed_manifest(key(), "job-1", "stage"));
  file("job-1", JobState::Staging, 40);
  cancel("job-1");
  EXPECT_EQ(sys_.cancels, std::vector<std::string>{"job-1"});
}

TEST_F(OtaManagerTest, CancelWorksBeforeAnApplyHasBegun) {
  start(signed_manifest(key(), "job-1", "apply"));
  cancel("job-1");
  EXPECT_EQ(sys_.cancels.size(), 1u);
}

TEST_F(OtaManagerTest, CancelNeverInterruptsAnInstall) {
  start(signed_manifest(key(), "job-1", "apply"));
  for (auto s : {JobState::Applying, JobState::Healthcheck}) {
    file("job-1", s, 50);
    cancel("job-1");
  }
  EXPECT_TRUE(sys_.cancels.empty());
  EXPECT_EQ(state_of("job-1"), JobState::Healthcheck);  // the refusal changes the detail, not the state
  EXPECT_NE(sink_.reports.back().detail.find("will not be interrupted"), std::string::npos);
}

TEST_F(OtaManagerTest, CancelOfFinishedOrUnknownJobsDoesNothing) {
  start(signed_manifest(key(), "job-1", "stage"));
  file("job-1", JobState::Staged, 100);
  manager_->poll();
  const auto reports = sink_.reports.size();
  cancel("job-1");
  cancel("never-heard-of-it");
  manager_->on_cancel_ota(nlohmann::json::array());
  EXPECT_TRUE(sys_.cancels.empty());
  EXPECT_EQ(sink_.reports.size(), reports);
}

TEST_F(OtaManagerTest, ACancelledJobIsReportedWhenTheUpdaterSaysSo) {
  start(signed_manifest(key(), "job-1", "stage"));
  cancel("job-1");
  file("job-1", JobState::Cancelled, 0, "cancelled: before downloading");
  manager_->poll();
  EXPECT_EQ(sink_.states().back(), JobState::Cancelled);
}

// ── Reconnect ────────────────────────────────────────────────────────────

TEST_F(OtaManagerTest, ConnectingReportsTheInstalledVersion) {
  manager_->on_connected();
  EXPECT_EQ(sink_.versions, std::vector<std::string>{"1.0.0"});
}

TEST_F(OtaManagerTest, AJobThatFinishedWhileOfflineIsReportedOnReconnect) {
  start(signed_manifest(key(), "job-1", "apply", "2.0.0"));
  manager_->detach();  // the tunnel dropped mid-update
  rebuild();           // ... and the agent restarted meanwhile
  manager_->detach();
  sys_.installed = "2.0.0";
  file("job-1", JobState::Success, 100, "healthy");

  FakeSink fresh;
  manager_->attach(&fresh);
  manager_->on_connected();

  ASSERT_FALSE(fresh.reports.empty());
  EXPECT_EQ(fresh.reports.back().job_id, "job-1");
  EXPECT_EQ(fresh.reports.back().state, JobState::Success);
  EXPECT_EQ(fresh.versions.front(), "2.0.0");
  EXPECT_EQ(state_of("job-1"), JobState::Success);
}

TEST_F(OtaManagerTest, StateChangesWhileDetachedAreKeptAndSentOnReconnect) {
  start(signed_manifest(key(), "job-1", "stage"));
  manager_->detach();
  file("job-1", JobState::Staged, 100, "ok");
  manager_->poll();  // e.g. a poll racing the disconnect
  EXPECT_FALSE(store_->find("job-1")->reported);

  FakeSink fresh;
  manager_->attach(&fresh);
  manager_->on_connected();
  EXPECT_EQ(fresh.states(), std::vector<JobState>{JobState::Staged});
  EXPECT_TRUE(store_->find("job-1")->reported);
}

TEST_F(OtaManagerTest, TheNewestJobIsResentEvenWhenItWasAlreadyReported) {
  start(signed_manifest(key(), "job-1", "stage"));
  file("job-1", JobState::Staged, 100);
  manager_->poll();  // reported over the old connection, which then died undelivered

  FakeSink fresh;
  manager_->attach(&fresh);
  manager_->on_connected();
  ASSERT_EQ(fresh.reports.size(), 1u);
  EXPECT_EQ(fresh.reports[0].state, JobState::Staged);
}

TEST_F(OtaManagerTest, OlderReportedJobsAreNotResent) {
  start(signed_manifest(key(), "job-1", "stage"));
  file("job-1", JobState::Staged, 100);
  manager_->poll();
  start(signed_manifest(key(), "job-2", "apply"));
  file("job-2", JobState::Success, 100);
  manager_->poll();

  FakeSink fresh;
  manager_->attach(&fresh);
  manager_->on_connected();
  ASSERT_EQ(fresh.reports.size(), 1u);
  EXPECT_EQ(fresh.reports[0].job_id, "job-2");
}

TEST_F(OtaManagerTest, GetOtaStatusReportsVersionAndNewestJob) {
  start(signed_manifest(key(), "job-1", "stage"));
  sink_.reports.clear();
  manager_->on_get_status();
  EXPECT_EQ(sink_.versions.size(), 1u);
  ASSERT_EQ(sink_.reports.size(), 1u);
  EXPECT_EQ(sink_.reports[0].job_id, "job-1");
}

// ── Isolation ────────────────────────────────────────────────────────────

TEST_F(OtaManagerTest, NothingThrowsOutOfAHandler) {
  sys_.throws = true;
  EXPECT_NO_THROW(start(signed_manifest(key(), "job-1", "stage")));
  EXPECT_NO_THROW(manager_->on_start_ota(nlohmann::json("garbage")));
  EXPECT_NO_THROW(manager_->on_cancel_ota(nlohmann::json(7)));
  EXPECT_NO_THROW(manager_->poll());
  EXPECT_NO_THROW(manager_->on_connected());
}
