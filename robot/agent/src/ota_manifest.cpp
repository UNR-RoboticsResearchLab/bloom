#include "ota_manifest.h"

#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/pem.h>

#include <algorithm>
#include <cctype>
#include <ctime>
#include <memory>

#include "base64.h"

namespace bloom_agent::ota {
namespace {

bool all_of_chars(std::string_view s, bool (*ok)(char)) {
  return std::all_of(s.begin(), s.end(), ok);
}
bool alnum(char c) { return std::isalnum(static_cast<unsigned char>(c)) != 0; }
bool id_char(char c) { return alnum(c) || c == '-' || c == '_'; }
bool version_char(char c) { return alnum(c) || c == '.' || c == '+' || c == '~' || c == '-'; }
bool suite_char(char c) { return alnum(c) || c == '.' || c == '-' || c == '_'; }
bool hex_char(char c) { return std::isxdigit(static_cast<unsigned char>(c)) != 0; }

std::optional<std::string> string_field(const nlohmann::json& obj, const char* key) {
  if (!obj.is_object() || !obj.contains(key) || !obj[key].is_string()) return std::nullopt;
  return obj[key].get<std::string>();
}

struct PkeyDeleter { void operator()(EVP_PKEY* p) const { EVP_PKEY_free(p); } };
struct BioDeleter { void operator()(BIO* b) const { BIO_free(b); } };
struct CtxDeleter { void operator()(EVP_MD_CTX* c) const { EVP_MD_CTX_free(c); } };

}  // namespace

const char* to_string(Action action) { return action == Action::Stage ? "stage" : "apply"; }

bool valid_job_id(std::string_view id) { return !id.empty() && id.size() <= 64 && all_of_chars(id, id_char); }

bool valid_version(std::string_view v) {
  return !v.empty() && v.size() <= 64 && std::isdigit(static_cast<unsigned char>(v.front())) && all_of_chars(v, version_char);
}

bool valid_suite(std::string_view s) { return !s.empty() && s.size() <= 64 && all_of_chars(s, suite_char); }

std::optional<Manifest> parse_manifest(const nlohmann::json& json, std::string& error, std::string& job_id_out) {
  job_id_out.clear();
  if (!json.is_object()) {
    error = "manifest is not a JSON object";
    return std::nullopt;
  }
  Manifest m;
  const auto job = string_field(json, "jobId");
  if (!job || !valid_job_id(*job)) {
    error = "jobId is missing or malformed";
    return std::nullopt;
  }
  m.job_id = job_id_out = *job;

  const auto action = string_field(json, "action");
  if (action == "stage") m.action = Action::Stage;
  else if (action == "apply") m.action = Action::Apply;
  else {
    error = "action must be \"stage\" or \"apply\"";
    return std::nullopt;
  }

  const auto version = string_field(json, "version");
  if (!version || !valid_version(*version)) {
    error = "version is missing or not a plain Debian version";
    return std::nullopt;
  }
  m.version = *version;

  if (!json.contains("source") || !json["source"].is_object()) {
    error = "source is missing";
    return std::nullopt;
  }
  const auto type = string_field(json["source"], "type");
  const auto suite = string_field(json["source"], "suite");
  if (!type || !suite) {
    error = "source.type and source.suite are required";
    return std::nullopt;
  }
  m.source_type = *type;
  m.suite = *suite;

  if (json.contains("sha256") && !json["sha256"].is_null()) {
    const auto sha = string_field(json, "sha256");
    if (!sha || sha->size() != 64 || !all_of_chars(*sha, hex_char)) {
      error = "sha256 must be 64 hex characters";
      return std::nullopt;
    }
    m.sha256 = *sha;
  }

  const auto signature = string_field(json, "signature");
  const auto expires = string_field(json, "expiresAt");
  if (!signature || signature->empty() || signature->size() > 512) {
    error = "signature is missing";
    return std::nullopt;
  }
  if (!expires) {
    error = "expiresAt is missing";
    return std::nullopt;
  }
  m.signature = *signature;
  m.expires_at = *expires;
  return m;
}

std::string signing_payload(const Manifest& m) {
  return "bloom-ota-v1\n" + m.job_id + "\n" + to_string(m.action) + "\n" + m.source_type + "\n" + m.suite + "\n" +
         m.version + "\n" + m.sha256 + "\n" + m.expires_at;
}

bool verify_signature(std::string_view public_key_pem, std::string_view payload, std::string_view signature_b64) {
  const auto signature = base64_decode(signature_b64);
  if (!signature || signature->size() != 64 || public_key_pem.empty()) return false;

  std::unique_ptr<BIO, BioDeleter> bio(BIO_new_mem_buf(public_key_pem.data(), static_cast<int>(public_key_pem.size())));
  if (!bio) return false;
  std::unique_ptr<EVP_PKEY, PkeyDeleter> key(PEM_read_bio_PUBKEY(bio.get(), nullptr, nullptr, nullptr));
  if (!key || EVP_PKEY_base_id(key.get()) != EVP_PKEY_ED25519) return false;

  std::unique_ptr<EVP_MD_CTX, CtxDeleter> ctx(EVP_MD_CTX_new());
  if (!ctx || EVP_DigestVerifyInit(ctx.get(), nullptr, nullptr, nullptr, key.get()) != 1) return false;
  return EVP_DigestVerify(ctx.get(), reinterpret_cast<const unsigned char*>(signature->data()), signature->size(),
                          reinterpret_cast<const unsigned char*>(payload.data()), payload.size()) == 1;
}

std::optional<std::int64_t> parse_utc_timestamp(std::string_view t) {
  // YYYY-MM-DDTHH:MM:SSZ
  if (t.size() != 20 || t[4] != '-' || t[7] != '-' || t[10] != 'T' || t[13] != ':' || t[16] != ':' || t[19] != 'Z') return std::nullopt;
  auto num = [&](std::size_t pos, std::size_t len) -> int {
    int v = 0;
    for (std::size_t i = pos; i < pos + len; ++i) {
      if (!std::isdigit(static_cast<unsigned char>(t[i]))) return -1;
      v = v * 10 + (t[i] - '0');
    }
    return v;
  };
  const int year = num(0, 4), mon = num(5, 2), day = num(8, 2), hour = num(11, 2), min = num(14, 2), sec = num(17, 2);
  if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 60)
    return std::nullopt;
  std::tm tm{};
  tm.tm_year = year - 1900;
  tm.tm_mon = mon - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = min;
  tm.tm_sec = sec;
  const std::time_t secs = timegm(&tm);
  if (secs == static_cast<std::time_t>(-1) || tm.tm_mday != day) return std::nullopt;  // e.g. Feb 30 normalises
  return static_cast<std::int64_t>(secs);
}

std::string validate(const Manifest& m, const Policy& policy, std::chrono::system_clock::time_point now) {
  if (m.source_type != "apt") return "unsupported source type \"" + m.source_type + "\"";
  if (!valid_suite(m.suite) || std::find(policy.allowed_suites.begin(), policy.allowed_suites.end(), m.suite) == policy.allowed_suites.end())
    return "suite is not allowed on this robot";

  const auto expiry = parse_utc_timestamp(m.expires_at);
  if (!expiry) return "expiresAt is not a UTC timestamp (YYYY-MM-DDTHH:MM:SSZ)";
  if (std::chrono::system_clock::to_time_t(now) >= *expiry) return "manifest has expired";

  if (policy.public_key_pem.empty()) return "no manifest signing key is configured on this robot";
  if (!verify_signature(policy.public_key_pem, signing_payload(m), m.signature)) return "manifest signature is invalid";
  return "";
}

}  // namespace bloom_agent::ota
