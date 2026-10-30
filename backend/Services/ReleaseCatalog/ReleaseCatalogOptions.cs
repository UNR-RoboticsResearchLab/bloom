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

        // Only tags starting with this are releases of the robot software (e.g. "robot-v"). After the
        // prefix (and an optional "v") the tag is the build's SemVer, e.g. robot-v1.3.0-rc.1, which
        // becomes package version 1.3.0~rc.1-1. Empty takes every tag. Tags that aren't SemVer are skipped.
        public string TagPrefix { get; set; } = "";

        // A GitHub pre-release whose SemVer matches this goes to "dev" instead of "canary".
        public string DevPattern { get; set; } = @"(^|[-.~+])dev";

        public int CacheMinutes { get; set; } = 10;
    }
}
