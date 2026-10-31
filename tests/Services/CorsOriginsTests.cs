using System;
using bloom.Services.Cors;
using Xunit;

namespace bloom.Tests.Services
{
    public class CorsOriginsTests
    {
        private static readonly string[] None = Array.Empty<string>();

        [Theory]
        [InlineData("http://localhost:3000")]
        [InlineData("http://127.0.0.1:3000")]
        [InlineData("http://[::1]:3000")]
        [InlineData("http://192.168.1.50:3000")]
        [InlineData("http://10.0.0.7:3000")]
        [InlineData("http://172.16.0.1:3000")]
        [InlineData("http://172.31.255.255:3000")]
        public void LocalAndPrivateOrigins_AreAllowedByDefault(string origin)
        {
            Assert.True(CorsOrigins.IsAllowed(origin, None));
        }

        [Theory]
        [InlineData("http://172.15.0.1:3000")]               // just outside 172.16/12
        [InlineData("http://172.32.0.1:3000")]
        [InlineData("http://100.64.1.2:3000")]               // Tailscale: not a private-LAN address
        [InlineData("https://bloom-box.tailnet.ts.net:3000")] // nor is a MagicDNS name
        [InlineData("https://example.com")]
        [InlineData("http://10.evil.com")]                   // host *names* that look like private IPs
        [InlineData("http://127.evil.com")]
        [InlineData("http://192.168.evil.com")]
        [InlineData("null")]                                 // opaque origins
        [InlineData("")]
        [InlineData(null)]
        public void OtherOrigins_AreRejectedByDefault(string? origin)
        {
            Assert.False(CorsOrigins.IsAllowed(origin, None));
        }

        [Fact]
        public void ListedOrigins_AreAllowed_ExactlyAndCaseInsensitively()
        {
            var extra = CorsOrigins.ParseList("http://bloom-box.tailnet.ts.net:3000, http://100.64.1.2:3000/");

            Assert.True(CorsOrigins.IsAllowed("http://bloom-box.tailnet.ts.net:3000", extra));
            Assert.True(CorsOrigins.IsAllowed("HTTP://Bloom-Box.Tailnet.TS.net:3000", extra));
            Assert.True(CorsOrigins.IsAllowed("http://100.64.1.2:3000", extra));

            // A listed origin is that origin only: not another port, scheme or host.
            Assert.False(CorsOrigins.IsAllowed("http://bloom-box.tailnet.ts.net:3001", extra));
            Assert.False(CorsOrigins.IsAllowed("https://bloom-box.tailnet.ts.net:3000", extra));
            Assert.False(CorsOrigins.IsAllowed("http://evil.ts.net:3000", extra));
        }

        [Theory]
        [InlineData(null, 0)]
        [InlineData("", 0)]
        [InlineData("  ", 0)]
        [InlineData("http://a:3000", 1)]
        [InlineData("http://a:3000,http://b:3000/;http://c:3000 http://d:3000", 4)]
        public void ParseList_SplitsAndTrims(string? list, int count)
        {
            Assert.Equal(count, CorsOrigins.ParseList(list).Length);
            Assert.DoesNotContain(CorsOrigins.ParseList(list), o => o.EndsWith("/"));
        }
    }
}
