#include "ota_state.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>

using namespace bloom_agent::ota;

TEST(OtaState, NamesRoundTripAndTerminalsAreRight) {
  for (const char* n : {"queued", "staging", "staged", "applying", "healthcheck", "success", "rolled_back", "failed", "deferred", "rejected", "cancelled"}) {
    const auto s = parse_job_state(n);
    ASSERT_TRUE(s) << n;
    EXPECT_STREQ(to_string(*s), n);
  }
  EXPECT_FALSE(parse_job_state("done"));
  for (auto s : {JobState::Queued, JobState::Staging, JobState::Applying, JobState::Healthcheck}) EXPECT_FALSE(is_terminal(s));
  for (auto s : {JobState::Staged, JobState::Success, JobState::RolledBack, JobState::Failed, JobState::Deferred, JobState::Rejected,
                 JobState::Cancelled})
    EXPECT_TRUE(is_terminal(s));
}

TEST(OtaState, ParsesTheUpdatersStateFile) {
  const auto s = parse_state_file(R"({"jobId":"j1","action":"apply","version":"2.0","state":"healthcheck","percent":80,"detail":"running","updatedAt":"x"})");
  ASSERT_TRUE(s);
  EXPECT_EQ(s->job_id, "j1");
  EXPECT_EQ(s->state, JobState::Healthcheck);
  EXPECT_EQ(s->percent, 80);
  EXPECT_EQ(s->detail, "running");
}

TEST(OtaState, RejectsGarbageAndSanitisesDetail) {
  EXPECT_FALSE(parse_state_file(""));
  EXPECT_FALSE(parse_state_file("{"));
  EXPECT_FALSE(parse_state_file(R"({"jobId":"j"})"));
  EXPECT_FALSE(parse_state_file(R"({"jobId":"j","state":"exploded"})"));
  const auto s = parse_state_file(R"({"jobId":"j","state":"failed","percent":900,"detail":"a\u0007b\nc"})");
  ASSERT_TRUE(s);
  EXPECT_EQ(s->percent, 100);
  EXPECT_EQ(s->detail, "abc");
  const auto big = parse_state_file(R"({"jobId":"j","state":"failed","detail":")" + std::string(1000, 'x') + R"("})");
  EXPECT_EQ(big->detail.size(), 300u);
}

class JobStoreFile : public ::testing::Test {
 protected:
  void SetUp() override {
    char tmpl[] = "/tmp/ota_store_XXXXXX";
    dir_ = mkdtemp(tmpl);
    path_ = dir_ + "/jobs.json";
  }
  void TearDown() override { (void)!system(("rm -rf " + dir_).c_str()); }
  std::string dir_, path_;
};

TEST_F(JobStoreFile, PersistsAcrossInstances) {
  {
    JobStore a(path_);
    a.add({{"j1", JobState::Staging, 40, "downloading"}, "stage", "1.0", 1234, true});
    ASSERT_TRUE(a.save());
  }
  JobStore b(path_);
  b.load();
  const auto* r = b.find("j1");
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(r->status.state, JobState::Staging);
  EXPECT_EQ(r->status.percent, 40);
  EXPECT_EQ(r->action, "stage");
  EXPECT_EQ(r->launched_at, 1234);
  EXPECT_TRUE(r->reported);
  EXPECT_EQ(b.active(), r);
  EXPECT_EQ(b.find("nope"), nullptr);
}

TEST_F(JobStoreFile, CorruptOrMissingFileIsAnEmptyStore) {
  JobStore none(dir_ + "/absent.json");
  none.load();
  EXPECT_TRUE(none.records().empty());
  std::ofstream(path_) << "{not json";
  JobStore bad(path_);
  bad.load();
  EXPECT_TRUE(bad.records().empty());
}

TEST(OtaJobStore, PrunesOldTerminalRecordsButKeepsRunningJobs) {
  JobStore s;
  s.add({{"running", JobState::Applying, 0, ""}, "apply", "1", 1, false});
  for (int i = 0; i < 80; ++i) s.add({{"done" + std::to_string(i), JobState::Success, 100, ""}, "apply", "1", 1, true});
  EXPECT_LE(s.records().size(), 50u);
  EXPECT_NE(s.find("running"), nullptr);
  EXPECT_NE(s.find("done79"), nullptr);
  EXPECT_EQ(s.find("done0"), nullptr);
  EXPECT_EQ(s.latest()->status.job_id, "done79");
}
