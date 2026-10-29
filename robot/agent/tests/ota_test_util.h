// Test helpers: a throwaway Ed25519 key that signs manifests the way the backend will.
#pragma once

#include <openssl/evp.h>
#include <openssl/pem.h>

#include <memory>
#include <string>

#include "base64.h"
#include "ota_manifest.h"

namespace ota_test {

struct TestKey {
  TestKey() {
    std::unique_ptr<EVP_PKEY_CTX, decltype(&EVP_PKEY_CTX_free)> ctx(EVP_PKEY_CTX_new_id(EVP_PKEY_ED25519, nullptr), EVP_PKEY_CTX_free);
    EVP_PKEY_keygen_init(ctx.get());
    EVP_PKEY_keygen(ctx.get(), &pkey);
    BIO* bio = BIO_new(BIO_s_mem());
    PEM_write_bio_PUBKEY(bio, pkey);
    char* data = nullptr;
    const long n = BIO_get_mem_data(bio, &data);
    public_pem.assign(data, static_cast<size_t>(n));
    BIO_free(bio);
  }
  ~TestKey() { EVP_PKEY_free(pkey); }

  std::string sign(const std::string& payload) const {
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    EVP_DigestSignInit(ctx.get(), nullptr, nullptr, nullptr, pkey);
    unsigned char sig[64];
    size_t len = sizeof(sig);
    EVP_DigestSign(ctx.get(), sig, &len, reinterpret_cast<const unsigned char*>(payload.data()), payload.size());
    return bloom_agent::base64_encode(std::string_view(reinterpret_cast<char*>(sig), len));
  }

  EVP_PKEY* pkey = nullptr;
  std::string public_pem;
};

// A manifest JSON object, correctly signed unless the caller tampers with it afterwards.
inline nlohmann::json signed_manifest(const TestKey& key, const std::string& job_id, const std::string& action,
                                      const std::string& version = "1.4.2-1jazzy", const std::string& suite = "jazzy-stable",
                                      const std::string& expires = "2999-01-01T00:00:00Z") {
  bloom_agent::ota::Manifest m;
  m.job_id = job_id;
  m.action = action == "apply" ? bloom_agent::ota::Action::Apply : bloom_agent::ota::Action::Stage;
  m.version = version;
  m.source_type = "apt";
  m.suite = suite;
  m.expires_at = expires;
  return {{"jobId", job_id},
          {"action", action},
          {"version", version},
          {"source", {{"type", "apt"}, {"suite", suite}}},
          {"signature", key.sign(bloom_agent::ota::signing_payload(m))},
          {"expiresAt", expires}};
}

}  // namespace ota_test
