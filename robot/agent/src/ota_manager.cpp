#include "ota_manager.h"

#include <exception>

#include "log.h"

namespace bloom_agent::ota {
namespace {
// systemd-run returns once the unit is queued; its state file appears a moment later. Don't call a
// job dead before it has had time to start.
constexpr std::int64_t kStartGraceSeconds = 20;

template <typename F>
void guarded(const char* what, F&& f) {
  try {
    f();
  } catch (const std::exception& e) {
    log::error(std::string("ota: ") + what + " failed: " + e.what());
  } catch (...) {
    log::error(std::string("ota: ") + what + " failed");
  }
}
}  // namespace

OtaManager::OtaManager(OtaConfig config, IOtaSystem& system, JobStore& store, KeyLoader key_loader, Clock clock)
    : cfg_(std::move(config)), system_(system), store_(store), key_loader_(std::move(key_loader)), clock_(std::move(clock)) {}

// One line per event, key=value, so `journalctl -u bloom-agent | grep job=<id>` follows a job.
void OtaManager::log_job(const std::string& job_id, const std::string& message, bool warn) const {
  const auto line = "ota job=" + job_id + " " + message;
  if (warn) log::warn(line);
  else log::info(line);
}

void OtaManager::report(JobRecord& record, JobStatus status) {
  const bool changed = !(record.status == status);
  record.status = std::move(status);
  record.reported = false;
  if (changed)
    log_job(record.status.job_id, std::string("state=") + to_string(record.status.state) + " percent=" +
                                      std::to_string(record.status.percent) + " detail=\"" + record.status.detail + "\"");
  if (sink_) {
    sink_->progress(record.status);
    record.reported = true;
    // The installed version changes only when an apply finishes one way or the other.
    if (record.status.state == JobState::Success || record.status.state == JobState::RolledBack)
      sink_->installed_version(system_.installed_version());
  }
  store_.save();
}

void OtaManager::send_current(JobRecord& record) {
  if (!sink_) return;
  sink_->progress(record.status);
  record.reported = true;
}

void OtaManager::on_connected() {
  guarded("on-connect catch-up", [&] {
    if (!sink_) return;
    sink_->installed_version(system_.installed_version());
    follow_active_job();
    JobRecord* newest = store_.latest();
    for (auto& r : store_.records())
      if (!r.reported || &r == newest) send_current(r);
    store_.save();
  });
}

void OtaManager::on_get_status() {
  guarded("GetOtaStatus", [&] {
    if (!sink_) return;
    sink_->installed_version(system_.installed_version());
    follow_active_job();
    if (JobRecord* newest = store_.latest()) send_current(*newest);
  });
}

void OtaManager::on_start_ota(const nlohmann::json& args) {
  guarded("StartOta", [&] {
    nlohmann::json manifest = args.is_array() && !args.empty() ? args[0] : nlohmann::json();
    if (manifest.is_string()) manifest = nlohmann::json::parse(manifest.get<std::string>(), nullptr, /*allow_exceptions=*/false);
    start(manifest);
  });
}

void OtaManager::start(const nlohmann::json& manifest_json) {
  std::string error, job_id;
  const auto manifest = parse_manifest(manifest_json, error, job_id);

  if (job_id.empty()) {
    // Without a usable id there is nothing to report against.
    log::warn("ota: ignoring a StartOta with no valid jobId: " + error);
    return;
  }
  if (store_.find(job_id)) {
    log_job(job_id, "duplicate StartOta ignored");
    return;
  }

  // From here the job id is known to us for good, whatever happens next.
  auto reject = [&](const std::string& reason, const std::string& action, const std::string& version) {
    log_job(job_id, "rejected: " + reason, /*warn=*/true);
    auto& rec = store_.add({{job_id, JobState::Rejected, 0, reason}, action, version, 0, false});
    report(rec, rec.status);
  };

  if (!manifest) return reject(error, "", "");

  Policy policy{cfg_.allowed_suites, key_loader_()};
  if (const auto reason = validate(*manifest, policy, clock_()); !reason.empty())
    return reject(reason, to_string(manifest->action), manifest->version);

  if (const auto* busy = store_.active()) return reject("another update (" + busy->status.job_id + ") is still in progress", to_string(manifest->action), manifest->version);

  auto& rec = store_.add({{job_id, JobState::Queued, 0, "accepted"}, to_string(manifest->action), manifest->version,
                          std::chrono::duration_cast<std::chrono::seconds>(clock_().time_since_epoch()).count(), false});
  // Persist before launching: a crash between the two must not allow a second launch of this id.
  store_.save();
  log_job(job_id, std::string("accepted action=") + rec.action + " version=" + rec.version);
  report(rec, rec.status);

  const auto launched = system_.launch({job_id, manifest->action, manifest->version});
  if (!launched.ok) report(rec, {job_id, JobState::Failed, 0, launched.message});
}

void OtaManager::on_cancel_ota(const nlohmann::json& args) {
  guarded("CancelOta", [&] {
    if (args.is_array() && !args.empty() && args[0].is_string()) cancel(args[0].get<std::string>());
    else log::warn("ota: ignoring CancelOta without a jobId");
  });
}

void OtaManager::cancel(const std::string& job_id) {
  JobRecord* rec = store_.find(job_id);
  if (!rec) return log_job(job_id, "CancelOta for an unknown job ignored", true);

  follow_active_job();  // decide on the freshest state
  const auto state = rec->status.state;
  if (is_terminal(state)) return log_job(job_id, std::string("CancelOta ignored: job already ") + to_string(state));

  // Cancellable: still queued, or a stage job (downloading is safe to abandon). Never an apply that
  // has begun installing: the updater itself enforces this, this is the early, explained answer.
  const bool before_install = state == JobState::Queued || (state == JobState::Staging && rec->action == "stage");
  if (!before_install) {
    log_job(job_id, std::string("CancelOta refused in state ") + to_string(state), true);
    auto status = rec->status;
    status.detail = "cancel refused: the install has begun and will not be interrupted";
    return report(*rec, status);
  }
  const auto res = system_.request_cancel(job_id);
  if (!res.ok) return log_job(job_id, res.message, true);
  log_job(job_id, "cancel requested; the updater stops at its next checkpoint");
  // The updater reports `cancelled` itself; poll() picks it up.
}

void OtaManager::follow_active_job() {
  JobRecord* rec = store_.active();
  if (!rec) return;

  if (const auto file = system_.read_state(); file && file->job_id == rec->status.job_id && !(*file == rec->status))
    report(*rec, *file);
  if (is_terminal(rec->status.state)) return;

  // The updater is gone but never finished (killed, host rebooted mid-job, systemd-run lost).
  const auto now = std::chrono::duration_cast<std::chrono::seconds>(clock_().time_since_epoch()).count();
  if (now - rec->launched_at >= kStartGraceSeconds && !system_.unit_active(rec->status.job_id)) {
    // One more look, in case it finished between the two reads above.
    if (const auto file = system_.read_state(); file && file->job_id == rec->status.job_id && is_terminal(file->state)) {
      report(*rec, *file);
      return;
    }
    report(*rec, {rec->status.job_id, JobState::Failed, rec->status.percent, "the updater stopped without reporting a result"});
  }
}

void OtaManager::poll() {
  guarded("poll", [&] {
    if (store_.active()) follow_active_job();
  });
}

}  // namespace bloom_agent::ota
