// On-demand OTA updates, agent side. The agent is only the trigger and the reporter: `fleet-ota`
// (deploy/fleet-ota) does the work in its own systemd unit, apt downloads and verifies the packages,
// and SignalR carries nothing but this small manifest and progress events.
//
//   server -> agent:  StartOta(manifest)  CancelOta(jobId)  GetOtaStatus()
//   agent -> server:  ReportOtaProgress(jobId, state, percent, detail)  ReportInstalledVersion(version)
//
// One OtaManager lives for the whole process and is attached to each tunnel connection in turn, so
// jobs and their idempotency record survive reconnects. Nothing here may throw into the tunnel: an
// OTA failure must never cost the robot its connection (and with it the remote shell).
#pragma once

#include <chrono>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>

#include "config.h"
#include "ota_manifest.h"
#include "ota_state.h"
#include "ota_system.h"

namespace bloom_agent::ota {

// Where reports go while a tunnel is up.
class IOtaSink {
 public:
  virtual ~IOtaSink() = default;
  virtual void progress(const JobStatus& status) = 0;
  virtual void installed_version(const std::string& version) = 0;
};

class OtaManager {
 public:
  using Clock = std::function<std::chrono::system_clock::time_point()>;
  using KeyLoader = std::function<std::string()>;  // the signing key PEM, read fresh per manifest ("" if absent)

  OtaManager(OtaConfig config, IOtaSystem& system, JobStore& store, KeyLoader key_loader, Clock clock);

  // While detached, state changes are recorded and sent on the next connection.
  void attach(IOtaSink* sink) { sink_ = sink; }
  void detach() { sink_ = nullptr; }

  // Reports the installed version, catches up on anything that happened while offline, and
  // re-sends the newest job's state (the server de-duplicates; a report sent into a connection that
  // then died would otherwise be lost).
  void on_connected();

  // `args` is the hub invocation's argument array.
  void on_start_ota(const nlohmann::json& args);
  void on_cancel_ota(const nlohmann::json& args);
  void on_get_status();

  // Called about once a second while connected. Cheap when no job is running.
  void poll();

 private:
  void start(const nlohmann::json& manifest_json);
  void cancel(const std::string& job_id);
  void report(JobRecord& record, JobStatus status);
  void send_current(JobRecord& record);
  void follow_active_job();
  void log_job(const std::string& job_id, const std::string& message, bool warn = false) const;

  OtaConfig cfg_;
  IOtaSystem& system_;
  JobStore& store_;
  KeyLoader key_loader_;
  Clock clock_;
  IOtaSink* sink_ = nullptr;
};

}  // namespace bloom_agent::ota
