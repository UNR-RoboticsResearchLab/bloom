// bloom
// ShellHubOriginCheck.cs
// Blocks cross-site WebSocket hijacking of /hubs/shell: browsers don't apply CORS to WebSockets, so a
// third-party page could otherwise open the admin's terminal with their cookie. A request that carries
// an Origin must come from this host or RemoteShell:AllowedOrigins; a WebSocket upgrade must carry one
// (browsers always send it). Plain same-origin GETs (long-polling/SSE fallback) may omit it.

using Microsoft.Extensions.Options;

namespace bloom.Services.RemoteShell
{
    public class ShellHubOriginCheck
    {
        private readonly RequestDelegate _next;
        private readonly IOptions<RemoteShellOptions> _options;
        private readonly ILogger<ShellHubOriginCheck> _logger;

        public ShellHubOriginCheck(RequestDelegate next, IOptions<RemoteShellOptions> options, ILogger<ShellHubOriginCheck> logger)
        {
            _next = next;
            _options = options;
            _logger = logger;
        }

        public async Task InvokeAsync(HttpContext context)
        {
            if (context.Request.Path.StartsWithSegments("/hubs/shell") && !IsAllowed(context))
            {
                _logger.LogWarning("Rejected shell connection from origin {Origin}", context.Request.Headers.Origin.ToString());
                context.Response.StatusCode = StatusCodes.Status403Forbidden;
                return;
            }
            await _next(context);
        }

        private bool IsAllowed(HttpContext context)
        {
            var origin = context.Request.Headers.Origin.ToString();
            if (string.IsNullOrEmpty(origin))
                return !context.Request.Headers.Upgrade.ToString().Contains("websocket", StringComparison.OrdinalIgnoreCase);

            if (_options.Value.AllowedOrigins.Contains(origin, StringComparer.OrdinalIgnoreCase))
                return true;
            return Uri.TryCreate(origin, UriKind.Absolute, out var uri)
                && string.Equals(uri.Authority, context.Request.Host.Value, StringComparison.OrdinalIgnoreCase);
        }
    }
}
