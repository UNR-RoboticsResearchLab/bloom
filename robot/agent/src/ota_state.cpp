#include "ota_state.h"

#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "log.h"

namespace bloom_agent::ota {
namespace {
constexpr std::size_t kMaxRecords = 50;
constexpr std::size_t kMaxDetail = 300;

struct Name { JobState state; const char* name; };
constexpr Name kNames[] = {
    {JobState::Queued, "queued"},     {JobState::Staging, "staging"},   {JobState::Staged, "staged"},
    {JobState::Applying, "applying"}, {JobState::Healthcheck, "healthcheck"}, {JobState::Success, "success"},
    {JobState::RolledBack, "rolled_back"}, {JobState::Failed, "failed"}, {JobState::Deferred, "deferred"},
    {JobState::Rejected, "rejected"}, {JobState::Cancelled, "cancelled"},
};

std::string clean_detail(std::string s) {
  s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) { return c < 0x20 || c == 0x7f; }), s.end());
  if (s.size() > kMaxDetail) s.resize(kMaxDetail);
  return s;
}
}  // namespace

const char* to_string(JobState state) {
  for (const auto& n : kNames)
    if (n.state == state) return n.name;
  return "failed";
}

std::optional<JobState> parse_job_state(std::string_view text) {
  for (const auto& n : kNames)
    if (text == n.name) return n.state;
  return std::nullopt;
}

bool is_terminal(JobState s) {
  return s != JobState::Queued && s != JobState::Staging && s != JobState::Applying && s != JobState::Healthcheck;
}

std::optional<JobStatus> parse_state_file(std::string_view text) {
  const auto j = nlohmann::json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
  if (!j.is_object() || !j.contains("jobId") || !j["jobId"].is_string() || !j.contains("state") || !j["state"].is_string())
    return std::nullopt;
  const auto state = parse_job_state(j["state"].get<std::string>());
  if (!state) return std::nullopt;
  JobStatus s;
  s.job_id = j["jobId"].get<std::string>();
  s.state = *state;
  if (j.contains("percent") && j["percent"].is_number_integer()) s.percent = std::clamp(j["percent"].get<int>(), 0, 100);
  if (j.contains("detail") && j["detail"].is_string()) s.detail = clean_detail(j["detail"].get<std::string>());
  return s;
}

void JobStore::load() {
  records_.clear();
  if (path_.empty()) return;
  std::ifstream in(path_);
  if (!in) return;
  std::stringstream ss;
  ss << in.rdbuf();
  const auto j = nlohmann::json::parse(ss.str(), nullptr, /*allow_exceptions=*/false);
  if (!j.is_object() || !j.contains("jobs") || !j["jobs"].is_array()) {
    log::warn("ota: job store " + path_ + " is unreadable; starting empty");
    return;
  }
  for (const auto& r : j["jobs"]) {
    if (!r.is_object() || !r.contains("jobId") || !r["jobId"].is_string()) continue;
    const auto state = parse_job_state(r.value("state", std::string()));
    if (!state) continue;
    JobRecord rec;
    rec.status = {r["jobId"].get<std::string>(), *state, r.value("percent", 0), clean_detail(r.value("detail", std::string()))};
    rec.action = r.value("action", std::string());
    rec.version = r.value("version", std::string());
    rec.launched_at = r.value("launchedAt", std::int64_t{0});
    rec.reported = r.value("reported", false);
    records_.push_back(std::move(rec));
  }
}

bool JobStore::save() const {
  if (path_.empty()) return true;
  nlohmann::json jobs = nlohmann::json::array();
  for (const auto& r : records_)
    jobs.push_back({{"jobId", r.status.job_id}, {"state", to_string(r.status.state)}, {"percent", r.status.percent},
                    {"detail", r.status.detail}, {"action", r.action}, {"version", r.version},
                    {"launchedAt", r.launched_at}, {"reported", r.reported}});
  const std::string tmp = path_ + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    out << nlohmann::json{{"jobs", jobs}}.dump() << "\n";
    out.flush();
    if (!out) {
      log::warn("ota: cannot write job store " + tmp);
      return false;
    }
  }
  if (std::rename(tmp.c_str(), path_.c_str()) != 0) {
    log::warn("ota: cannot replace job store " + path_);
    return false;
  }
  return true;
}

JobRecord* JobStore::find(std::string_view job_id) {
  for (auto& r : records_)
    if (r.status.job_id == job_id) return &r;
  return nullptr;
}

JobRecord& JobStore::add(JobRecord record) {
  records_.push_back(std::move(record));
  // Keep the newest kMaxRecords, but never drop a job that is still running.
  while (records_.size() > kMaxRecords) {
    const auto it = std::find_if(records_.begin(), records_.end() - 1, [](const JobRecord& r) { return is_terminal(r.status.state); });
    if (it == records_.end() - 1) break;
    records_.erase(it);
  }
  return records_.back();
}

JobRecord* JobStore::active() {
  for (auto& r : records_)
    if (!is_terminal(r.status.state)) return &r;
  return nullptr;
}

JobRecord* JobStore::latest() { return records_.empty() ? nullptr : &records_.back(); }

}  // namespace bloom_agent::ota
