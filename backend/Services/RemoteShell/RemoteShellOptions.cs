// bloom
// RemoteShellOptions.cs
// Configuration for the fleet remote shell (section "RemoteShell").

namespace bloom.Services.RemoteShell
{
    public class RemoteShellOptions
    {
        public const string SectionName = "RemoteShell";

        // Off by default. Turning it on also requires a real Jwt:Key (see Program.cs).
        public bool Enabled { get; set; }

        // Browser origins allowed to open /hubs/shell besides the backend's own host, e.g. the CRA
        // dev server. Blocks cross-site WebSocket hijacking with an admin's cookie.
        public string[] AllowedOrigins { get; set; } = Array.Empty<string>();

        // Per admin, sliding one-minute window.
        public int SessionOpensPerMinute { get; set; } = 5;

        // The agent pings every ~25s; a link silent for this long is treated as dead.
        public int AgentIdleTimeoutSeconds { get; set; } = 75;

        // How long to wait for an agent to confirm it applied a pushed secret.
        public int SecretPushTimeoutSeconds { get; set; } = 30;
    }
}
