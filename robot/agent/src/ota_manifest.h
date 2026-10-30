// OTA update manifest: parsing and validation. A manifest arrives over the SignalR hub as a small
// JSON object (StartOta) and is trusted only after every check here passes. Nothing from it is ever
// interpolated into a shell command: the strict patterns below are defence in depth on top of the
// launcher passing arguments as an array (ota_system.h).
//
//   { "jobId": "...", "action": "stage"|"apply", "version": "1.4.2-1jazzy",
//     "source": { "type": "apt", "suite": "jazzy-stable" },
//     "sha256": "<64 hex, optional, reserved for bundle mode>",
//     "signature": "<base64 Ed25519>", "expiresAt": "2026-11-01T12:00:00Z" }
//
// The signature covers signing_payload(): a fixed text form, so no JSON canonicalisation is needed.
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

namespace bloom_agent::ota {

enum class Action { Stage, Apply };
const char* to_string(Action action);

struct Manifest {
  std::string job_id;
  Action action = Action::Stage;
  std::string version;
  std::string source_type;
  std::string suite;
  std::string sha256;      // empty when absent
  std::string signature;   // base64
  std::string expires_at;  // as received, UTC "YYYY-MM-DDTHH:MM:SSZ"
};

// ids are used in systemd unit names and file names: letters, digits, '-' and '_' only.
bool valid_job_id(std::string_view id);
// A Debian version without an epoch: starts with a digit, then [A-Za-z0-9.+~-], at most 64 chars.
bool valid_version(std::string_view version);
bool valid_suite(std::string_view suite);

// nullopt (with `error` filled in) when a field is missing, has the wrong type or is malformed.
// The job id is recovered into `job_id_out` whenever it is itself valid, so a rejection can be
// reported against the right job.
std::optional<Manifest> parse_manifest(const nlohmann::json& json, std::string& error, std::string& job_id_out);

// "bloom-ota-v1\n<jobId>\n<action>\n<type>\n<suite>\n<version>\n<sha256>\n<expiresAt>"
std::string signing_payload(const Manifest& manifest);

// Ed25519 over `payload`. `public_key_pem` is an SPKI PEM; the signature is base64. False for any
// failure, including an unreadable key.
bool verify_signature(std::string_view public_key_pem, std::string_view payload, std::string_view signature_b64);

struct Policy {
  std::vector<std::string> allowed_suites;
  std::string public_key_pem;  // empty = not configured, which rejects everything
};

// Seconds since the epoch for "YYYY-MM-DDTHH:MM:SSZ"; nullopt for any other shape.
std::optional<std::int64_t> parse_utc_timestamp(std::string_view text);

// Empty string when the manifest is acceptable, else a reason fit for the dashboard. Checks, in
// order: source type, suite allowlist, expiry, then the signature (cheapest first).
std::string validate(const Manifest& manifest, const Policy& policy, std::chrono::system_clock::time_point now);

}  // namespace bloom_agent::ota
