using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using bloom.Services.ReleaseCatalog;
using Microsoft.Extensions.Caching.Memory;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using Moq;
using Xunit;

namespace bloom.Tests.Services
{
    public class ReleaseCatalogServiceTests
    {
        private sealed class StubHandler : HttpMessageHandler
        {
            public Func<HttpRequestMessage, HttpResponseMessage> Respond { get; set; } =
                _ => new HttpResponseMessage(HttpStatusCode.OK) { Content = Json("[]") };
            public List<HttpRequestMessage> Requests { get; } = new();

            protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken ct)
            {
                Requests.Add(request);
                return Task.FromResult(Respond(request));
            }
        }

        private static StringContent Json(string json) => new(json, Encoding.UTF8, "application/json");

        private static string Release(string tag, bool prerelease = false, bool draft = false) =>
            $"{{\"tag_name\":\"{tag}\",\"prerelease\":{prerelease.ToString().ToLowerInvariant()},\"draft\":{draft.ToString().ToLowerInvariant()}}}";

        private static (ReleaseCatalogService Service, StubHandler Handler) Create(ReleaseCatalogOptions options)
        {
            var handler = new StubHandler();
            var factory = new Mock<IHttpClientFactory>();
            factory.Setup(f => f.CreateClient(ReleaseCatalogService.HttpClientName))
                .Returns(() => new HttpClient(handler, disposeHandler: false) { BaseAddress = new Uri("https://api.github.com/") });
            var monitor = Mock.Of<IOptionsMonitor<ReleaseCatalogOptions>>(m => m.CurrentValue == options);
            var service = new ReleaseCatalogService(
                factory.Object, monitor, new MemoryCache(new MemoryCacheOptions()), NullLogger<ReleaseCatalogService>.Instance);
            return (service, handler);
        }

        private static ReleaseCatalogOptions GitHub(string tagPrefix = "", string token = "") => new()
        {
            GitHub = new GitHubReleaseOptions { Repository = "org/robot", TagPrefix = tagPrefix, Token = token },
        };

        [Fact]
        public async Task ConfiguredVersions_AreSortedNewestFirst_InEveryChannel()
        {
            var (service, handler) = Create(new ReleaseCatalogOptions
            {
                Channels = new()
                {
                    ["prod"] = new[] { "1.0.9-1", "1.0.20-1", "1.0.20~gabc-1", "not a version" },
                    ["Canary"] = new[] { "1.1.0-1" },
                    ["nightly"] = new[] { "9.9.9" }, // unknown channel: ignored
                },
            });

            var catalog = await service.GetCatalogAsync();

            Assert.Equal(new[] { "1.0.20-1", "1.0.20~gabc-1", "1.0.9-1" }, catalog.Channels["prod"]);
            Assert.Equal(new[] { "1.1.0-1" }, catalog.Channels["canary"]);
            Assert.Empty(catalog.Channels["dev"]);
            Assert.Equal(3, catalog.Channels.Count);
            Assert.Empty(handler.Requests); // no repository configured, so GitHub isn't asked
            Assert.Empty(catalog.Warnings);
        }

        [Fact]
        public async Task GitHubReleases_AreMappedToChannels_AndFilteredByTagPrefix()
        {
            var (service, handler) = Create(GitHub(tagPrefix: "robot-v"));
            handler.Respond = _ => new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = Json("[" + string.Join(",",
                    Release("robot-v1.2.0"),
                    Release("robot-v1.3.0-rc.1", prerelease: true),
                    Release("robot-v1.3.0-dev.5", prerelease: true),
                    Release("robot-v1.4.0", draft: true),     // drafts aren't published
                    Release("0.2.0"),                          // another project's tag
                    Release("robot-vlatest")) + "]"),         // not SemVer
            };

            var catalog = await service.GetCatalogAsync();

            Assert.Equal(new[] { "1.2.0-1" }, catalog.Channels["prod"]);
            Assert.Equal(new[] { "1.3.0~rc.1-1" }, catalog.Channels["canary"]);
            Assert.Equal(new[] { "1.3.0~dev.5-1" }, catalog.Channels["dev"]);
            var request = Assert.Single(handler.Requests);
            Assert.Equal("https://api.github.com/repos/org/robot/releases?per_page=100", request.RequestUri!.ToString());
            Assert.Null(request.Headers.Authorization);
        }

        [Fact]
        public async Task GitHubAndConfig_AreMerged_WithoutDuplicates()
        {
            var options = GitHub(token: "ghp_test");
            options.Channels["prod"] = new[] { "1.2.0-1", "1.1.0-1" };
            var (service, handler) = Create(options);
            handler.Respond = _ => new HttpResponseMessage(HttpStatusCode.OK)
            {
                Content = Json("[" + Release("v1.2.0") + "," + Release("v1.10.0") + "]"),
            };

            var catalog = await service.GetCatalogAsync();

            Assert.Equal(new[] { "1.10.0-1", "1.2.0-1", "1.1.0-1" }, catalog.Channels["prod"]);
            Assert.Equal("Bearer", handler.Requests.Single().Headers.Authorization!.Scheme);
        }

        [Fact]
        public async Task GitHubResult_IsCached()
        {
            var (service, handler) = Create(GitHub());
            handler.Respond = _ => new HttpResponseMessage(HttpStatusCode.OK) { Content = Json("[" + Release("1.0.0") + "]") };

            await service.GetCatalogAsync();
            var second = await service.GetCatalogAsync();

            Assert.Single(handler.Requests);
            Assert.Equal(new[] { "1.0.0-1" }, second.Channels["prod"]);
        }

        [Fact]
        public async Task GitHubFailure_KeepsConfiguredVersions_AndWarns()
        {
            var options = GitHub();
            options.Channels["prod"] = new[] { "1.0.0-1" };
            var (service, handler) = Create(options);
            handler.Respond = _ => new HttpResponseMessage(HttpStatusCode.Forbidden) { Content = Json("{\"message\":\"rate limited\"}") };

            var catalog = await service.GetCatalogAsync();

            Assert.Equal(new[] { "1.0.0-1" }, catalog.Channels["prod"]);
            Assert.Contains("org/robot", Assert.Single(catalog.Warnings));
        }
    }

    public class DebianVersionTests
    {
        [Theory]
        [InlineData("1.0.20-1", "1.0.9-1", 1)]
        [InlineData("1.0.20~gabc-1", "1.0.20-1", -1)]   // ~ sorts before the end of the string
        [InlineData("1.0.20-2", "1.0.20-1", 1)]
        [InlineData("1.0.20+b1", "1.0.20", 1)]
        [InlineData("1.0a", "1.0", 1)]
        [InlineData("1.0a", "1.0+", -1)]                 // letters before other symbols
        [InlineData("1.010", "1.10", 0)]                 // leading zeros don't count
        [InlineData("1.4.2-1jazzy", "1.4.2-1jazzy", 0)]
        public void Compare_MatchesDpkg(string a, string b, int expectedSign)
        {
            Assert.Equal(expectedSign, Math.Sign(DebianVersion.Compare(a, b)));
            Assert.Equal(-expectedSign, Math.Sign(DebianVersion.Compare(b, a)));
        }

        [Theory]
        [InlineData("1.0.20", "1.0.20-1")]
        [InlineData("1.0.20-g4fc39fd", "1.0.20~g4fc39fd-1")]
        [InlineData("1.3.0-rc-1.2", "1.3.0~rc.1.2-1")]   // a "-" inside the pre-release would read as the revision
        [InlineData("1.0.20+build-5", "1.0.20+build.5-1")]
        [InlineData("1.0", null)]
        [InlineData("1.0.20-1jazzy-2", "1.0.20~1jazzy.2-1")]
        [InlineData("latest", null)]
        public void FromSemVer_MatchesThePackagingTool(string semver, string? expected)
        {
            Assert.Equal(expected, DebianVersion.FromSemVer(semver));
        }

        [Theory]
        [InlineData("1.4.2-1jazzy", true)]
        [InlineData("1.0.20~gabc-1", true)]
        [InlineData("v1.0", false)]
        [InlineData("1:1.0", false)]  // no epochs
        [InlineData("", false)]
        public void IsValid_MatchesTheAgentsPattern(string version, bool valid)
        {
            Assert.Equal(valid, DebianVersion.IsValid(version));
        }
    }
}
