// bloom
// ReleaseCatalogService.cs
// Builds the per-channel list of OTA versions for the fleet view from two sources: versions listed
// in config (Releases:Channels) and the GitHub Releases of Releases:GitHub:Repository. GitHub is
// read server-side (the token stays here, and one cached call serves every admin), and a GitHub
// failure leaves the configured versions plus a warning rather than an error.

using System.Net.Http.Headers;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.RegularExpressions;
using bloom.Models.dto;
using Microsoft.Extensions.Caching.Memory;
using Microsoft.Extensions.Options;

namespace bloom.Services.ReleaseCatalog
{
    public interface IReleaseCatalogService
    {
        Task<ReleaseCatalogDto> GetCatalogAsync(CancellationToken ct = default);
    }

    public class ReleaseCatalogService : IReleaseCatalogService
    {
        public const string HttpClientName = "GitHubReleases";
        public static readonly string[] ChannelNames = { "prod", "canary", "dev" };

        // A failed fetch is retried sooner than a good one is refreshed, but not on every poll.
        private static readonly TimeSpan FailureCacheTime = TimeSpan.FromMinutes(1);

        private readonly IHttpClientFactory _httpClientFactory;
        private readonly IOptionsMonitor<ReleaseCatalogOptions> _options;
        private readonly IMemoryCache _cache;
        private readonly ILogger<ReleaseCatalogService> _logger;

        public ReleaseCatalogService(
            IHttpClientFactory httpClientFactory,
            IOptionsMonitor<ReleaseCatalogOptions> options,
            IMemoryCache cache,
            ILogger<ReleaseCatalogService> logger)
        {
            _httpClientFactory = httpClientFactory;
            _options = options;
            _cache = cache;
            _logger = logger;
        }

        public async Task<ReleaseCatalogDto> GetCatalogAsync(CancellationToken ct = default)
        {
            var options = _options.CurrentValue;
            var channels = ChannelNames.ToDictionary(c => c, _ => new HashSet<string>());
            var warnings = new List<string>();

            foreach (var (channel, versions) in options.Channels)
            {
                if (!channels.TryGetValue(channel.ToLowerInvariant(), out var set))
                {
                    _logger.LogWarning("Releases:Channels has unknown channel {Channel}; expected prod, canary or dev", channel);
                    continue;
                }
                foreach (var v in versions ?? Array.Empty<string>())
                {
                    if (DebianVersion.IsValid(v?.Trim())) set.Add(v!.Trim());
                    else _logger.LogWarning("Ignoring configured release {Version}: not a valid package version", v);
                }
            }

            if (!string.IsNullOrWhiteSpace(options.GitHub.Repository))
            {
                var github = await GetGitHubReleasesAsync(options.GitHub, ct);
                if (github.Error != null) warnings.Add(github.Error);
                foreach (var (channel, version) in github.Releases) channels[channel].Add(version);
            }

            return new ReleaseCatalogDto
            {
                Channels = channels.ToDictionary(
                    kv => kv.Key,
                    kv => kv.Value.OrderByDescending(v => v, Comparer<string>.Create(DebianVersion.Compare)).ToList()),
                Warnings = warnings,
            };
        }

        private record GitHubResult(List<(string Channel, string Version)> Releases, string? Error);

        private async Task<GitHubResult> GetGitHubReleasesAsync(GitHubReleaseOptions gh, CancellationToken ct)
        {
            // Keyed on the settings that shape the result, so a config reload doesn't serve stale lists.
            var cacheKey = $"release-catalog:github:{gh.Repository}:{gh.TagPrefix}:{gh.DevPattern}";
            if (_cache.TryGetValue(cacheKey, out GitHubResult? cached) && cached != null) return cached;

            GitHubResult result;
            try
            {
                result = new GitHubResult(await FetchAsync(gh, ct), null);
            }
            catch (Exception ex) when (ex is HttpRequestException or JsonException or TaskCanceledException && !ct.IsCancellationRequested)
            {
                _logger.LogWarning(ex, "Couldn't read GitHub releases of {Repository}", gh.Repository);
                result = new GitHubResult(new(), $"Couldn't read GitHub releases of {gh.Repository}.");
            }

            var ttl = result.Error == null ? TimeSpan.FromMinutes(Math.Max(1, gh.CacheMinutes)) : FailureCacheTime;
            _cache.Set(cacheKey, result, ttl);
            return result;
        }

        private async Task<List<(string, string)>> FetchAsync(GitHubReleaseOptions gh, CancellationToken ct)
        {
            var client = _httpClientFactory.CreateClient(HttpClientName);
            using var request = new HttpRequestMessage(HttpMethod.Get, $"repos/{gh.Repository.Trim()}/releases?per_page=100");
            if (!string.IsNullOrWhiteSpace(gh.Token))
            {
                request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", gh.Token);
            }

            using var response = await client.SendAsync(request, ct);
            response.EnsureSuccessStatusCode();
            var releases = await response.Content.ReadFromJsonAsync<List<GitHubRelease>>(cancellationToken: ct) ?? new();

            var dev = new Regex(gh.DevPattern, RegexOptions.IgnoreCase, TimeSpan.FromMilliseconds(100));
            var found = new List<(string, string)>();
            foreach (var release in releases)
            {
                if (release.Draft || string.IsNullOrEmpty(release.TagName)) continue;
                if (!release.TagName.StartsWith(gh.TagPrefix, StringComparison.Ordinal)) continue;

                // Tags carry the SemVer the build was versioned with (git forbids "~" in tag names);
                // the robot's package version is its Debian form.
                var semver = release.TagName[gh.TagPrefix.Length..];
                if (semver.StartsWith('v') || semver.StartsWith('V')) semver = semver[1..];
                var version = DebianVersion.FromSemVer(semver);
                if (version == null || !DebianVersion.IsValid(version)) continue; // e.g. a demo tag

                var channel = !release.Prerelease ? "prod" : dev.IsMatch(semver) ? "dev" : "canary";
                found.Add((channel, version));
            }
            return found;
        }

        private class GitHubRelease
        {
            [JsonPropertyName("tag_name")] public string TagName { get; set; } = "";
            [JsonPropertyName("draft")] public bool Draft { get; set; }
            [JsonPropertyName("prerelease")] public bool Prerelease { get; set; }
        }
    }
}
