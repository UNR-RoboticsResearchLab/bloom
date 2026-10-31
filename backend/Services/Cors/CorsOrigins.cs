// bloom
// CorsOrigins.cs
// Which browser origins may call the API with credentials: localhost, loopback and private-LAN
// addresses (development), plus any origins listed in Cors:AllowedOrigins (comma separated), e.g. the
// Tailscale address a dev frontend is opened from, which is not a private-LAN address.

using System.Net;
using System.Net.Sockets;

namespace bloom.Services.Cors
{
    public static class CorsOrigins
    {
        // "http://a:3000, http://b:3000/" -> ["http://a:3000", "http://b:3000"]
        public static string[] ParseList(string? list) =>
            (list ?? "")
                .Split(new[] { ',', ';', ' ' }, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
                .Select(o => o.TrimEnd('/'))
                .ToArray();

        public static bool IsAllowed(string? origin, IEnumerable<string> extraOrigins)
        {
            if (string.IsNullOrWhiteSpace(origin) || !Uri.TryCreate(origin, UriKind.Absolute, out var uri))
                return false;

            if (extraOrigins.Contains(origin.TrimEnd('/'), StringComparer.OrdinalIgnoreCase))
                return true;

            if (uri.Host == "localhost")
                return true;

            // Only real IP addresses: a host *name* that merely starts with "10." or "192.168." is not local.
            return IPAddress.TryParse(uri.Host, out var ip) && IsLoopbackOrPrivate(ip);
        }

        private static bool IsLoopbackOrPrivate(IPAddress ip)
        {
            if (IPAddress.IsLoopback(ip)) return true;
            if (ip.AddressFamily != AddressFamily.InterNetwork) return false;

            var b = ip.GetAddressBytes();
            return b[0] == 10                                  // 10.0.0.0/8
                || (b[0] == 172 && b[1] >= 16 && b[1] <= 31)   // 172.16.0.0/12
                || (b[0] == 192 && b[1] == 168);               // 192.168.0.0/16
        }
    }
}
