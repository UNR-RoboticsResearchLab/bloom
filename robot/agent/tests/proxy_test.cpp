#include "proxy.h"

#include "base64.h"

#include <gtest/gtest.h>

#include <map>

using namespace bloom_agent;

namespace {
EnvLookup env(std::map<std::string, std::string> vars) {
  auto shared = std::make_shared<std::map<std::string, std::string>>(std::move(vars));
  return [shared](const char* name) -> const char* {
    auto it = shared->find(name);
    return it == shared->end() ? nullptr : it->second.c_str();
  };
}
}  // namespace

TEST(Proxy, NoneConfiguredMeansDirect) { EXPECT_FALSE(proxy_for("bloom.example.edu", env({}))); }

TEST(Proxy, ParsesHttpsProxyWithCredentials) {
  auto p = proxy_for("bloom.example.edu", env({{"HTTPS_PROXY", "http://user:p%40ss@proxy.uni.edu:3128/"}}));
  ASSERT_TRUE(p);
  EXPECT_EQ(p->host, "proxy.uni.edu");
  EXPECT_EQ(p->port, "3128");
  EXPECT_EQ(p->authorization, "Basic " + base64_encode("user:p@ss"));
  EXPECT_EQ(p->source, "HTTPS_PROXY");
}

TEST(Proxy, AcceptsBareHostPortAndLowercaseVar) {
  auto p = proxy_for("bloom.example.edu", env({{"https_proxy", "proxy:8080"}}));
  ASSERT_TRUE(p);
  EXPECT_EQ(p->host, "proxy");
  EXPECT_EQ(p->port, "8080");
  EXPECT_TRUE(p->authorization.empty());
}

TEST(Proxy, RejectsUnsupportedSchemes) {
  EXPECT_FALSE(parse_proxy_url("socks5://proxy:1080"));
  EXPECT_FALSE(parse_proxy_url("https://proxy:443"));
}

TEST(Proxy, NoProxyMatching) {
  EXPECT_TRUE(no_proxy_matches("bloom.example.edu", "localhost,example.edu"));
  EXPECT_TRUE(no_proxy_matches("bloom.example.edu", " .example.edu "));
  EXPECT_TRUE(no_proxy_matches("example.edu", ".example.edu"));
  EXPECT_TRUE(no_proxy_matches("anything", "*"));
  EXPECT_TRUE(no_proxy_matches("BLOOM.Example.EDU", "example.edu"));
  EXPECT_FALSE(no_proxy_matches("notexample.edu", "example.edu"));
  EXPECT_FALSE(no_proxy_matches("bloom.example.edu", "other.edu,,"));
}

TEST(Proxy, NoProxyWinsOverHttpsProxy) {
  EXPECT_FALSE(proxy_for("bloom.example.edu", env({{"HTTPS_PROXY", "proxy:3128"}, {"NO_PROXY", "example.edu"}})));
}

TEST(Proxy, ConnectRequestIsWellFormed) {
  ProxyConfig p{"proxy", "3128", "Basic dXNlcjpwYXNz", "HTTPS_PROXY"};
  EXPECT_EQ(connect_request("bloom.example.edu", "443", p),
            "CONNECT bloom.example.edu:443 HTTP/1.1\r\n"
            "Host: bloom.example.edu:443\r\n"
            "Proxy-Authorization: Basic dXNlcjpwYXNz\r\n"
            "User-Agent: bloom-robot-agent\r\n\r\n");
}
