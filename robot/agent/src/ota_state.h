// OTA job states, the state file fleet-ota writes, and the agent's persistent record of jobs.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bloom_agent::ota {

//   queued -> staging -> staged                                  (stage)
//   queued -> applying -> healthcheck -> success | rolled_back   (apply)
//   ... any time: failed | deferred | rejected | cancelled
enum class JobState { Queued, Staging, Staged, Applying, Healthcheck, Success, RolledBack, Failed, Deferred, Rejected, Cancelled };

const char* to_string(JobState state);
std::optional<JobState> parse_job_state(std::string_view text);
// Nothing more will happen to the job. `staged` counts: a stage job ends there.
bool is_terminal(JobState state);

struct JobStatus {
  std::string job_id;
  JobState state = JobState::Queued;
  int percent = 0;
  std::string detail;
  bool operator==(const JobStatus& o) const {
    return job_id == o.job_id && state == o.state && percent == o.percent && detail == o.detail;
  }
};

// Parses /var/lib/fleet-ota/state.json. nullopt for anything that is not a complete status. The
// detail is cut to 300 characters with control characters removed: it is relayed to a dashboard.
std::optional<JobStatus> parse_state_file(std::string_view text);

// What the agent remembers about each job it has accepted or rejected. Its existence is the
// idempotency key: a jobId in the store is never started again.
struct JobRecord {
  JobStatus status;
  std::string action;          // "stage" | "apply"
  std::string version;
  std::int64_t launched_at = 0;  // epoch seconds; 0 = never launched (rejected)
  bool reported = false;         // latest status was handed to a live connection
};

class JobStore {
 public:
  // An empty path keeps everything in memory (tests).
  explicit JobStore(std::string path = "") : path_(std::move(path)) {}

  // Reads the file; a missing file is an empty store and a corrupt one is too (it is logged).
  void load();
  // Writes atomically (temp file + rename). False if it could not be written.
  bool save() const;

  JobRecord* find(std::string_view job_id);
  JobRecord& add(JobRecord record);  // appends and prunes old terminal records
  JobRecord* active();               // the one non-terminal job, if any
  JobRecord* latest();               // most recently added
  std::vector<JobRecord>& records() { return records_; }

 private:
  std::string path_;
  std::vector<JobRecord> records_;
};

}  // namespace bloom_agent::ota
