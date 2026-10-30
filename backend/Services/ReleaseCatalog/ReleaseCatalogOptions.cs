// bloom
// ReleaseCatalogOptions.cs
// Configuration for the OTA release list the fleet view offers (section "Releases").

namespace bloom.Services.ReleaseCatalog
{
    public class ReleaseCatalogOptions
    {
        public const string SectionName = "Releases";

        // Versions listed by hand, per channel ("prod", "canary", "dev"). Merged with GitHub's.
        public Dictionary<string, string[]> Channels { get; set; } = new();

        public GitHubReleaseOptions GitHub { get; set; } = new();
    }

    public class GitHubReleaseOptions
    {
        // "owner/repo". Empty turns the GitHub source off.
        public string Repository { get; set; } = "";

        // Optional; needed for a private repo, and raises the 60 requests/hour anonymous limit.
        public string Token { get; set; } = "";

        // Only tags starting with this are releases of the robot software (e.g. "robot-v"); the prefix
        // is stripped, then a leading "v". Empty takes every tag.
        public string TagPrefix { get; set; } = "";

        // A pre-release whose version matches this goes to "dev" instead of "canary".
        public string DevPattern { get; set; } = @"(^|[-.~+])dev";

        public int CacheMinutes { get; set; } = 10;
    }
}
