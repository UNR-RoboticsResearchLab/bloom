// bloom
// AdminConnection.cs
// Handles one admin's browser terminal on /ws/shell. One socket = one shell session, opened from
// the query string: /ws/shell?robotId=...&cols=80&rows=24&force=false. The endpoint's
// "AdminOnly" policy has already required an Admin cookie; this re-checks the Origin header so a
// third-party page can't ride that cookie (cross-site WebSocket hijacking).

using System.Net.WebSockets;
using System.Security.Claims;
using System.Text.Json;
using bloom.Data;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Options;

namespace bloom.Services.RemoteShell
{
    public class AdminConnection
    {
        private readonly IShellRelay _relay;
        private readonly BloomDbContext _db;
        private readonly IOptions<RemoteShellOptions> _options;
        private readonly ILogger<AdminConnection> _logger;

        public AdminConnection(
            IShellRelay relay,
            BloomDbContext db,
            IOptions<RemoteShellOptions> options,
            ILogger<AdminConnection> logger)
        {
            _relay = relay;
            _db = db;
            _options = options;
            _logger = logger;
        }

        public async Task RunAsync(HttpContext context)
        {
            if (!context.WebSockets.IsWebSocketRequest)
            {
                context.Response.StatusCode = StatusCodes.Status400BadRequest;
                return;
            }
            if (!IsAllowedOrigin(context))
            {
                _logger.LogWarning("Rejected shell connection from origin {Origin}", context.Request.Headers.Origin.ToString());
                context.Response.StatusCode = StatusCodes.Status403Forbidden;
                return;
            }

            var query = context.Request.Query;
            if (!Guid.TryParse(query["robotId"], out var robotId))
            {
                context.Response.StatusCode = StatusCodes.Status400BadRequest;
                return;
            }
            var robotName = await _db.Robots.Where(r => r.Id == robotId).Select(r => r.Name).FirstOrDefaultAsync();
            if (robotName == null)
            {
                context.Response.StatusCode = StatusCodes.Status404NotFound;
                return;
            }

            var adminUserId = context.User.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (adminUserId == null)
            {
                context.Response.StatusCode = StatusCodes.Status403Forbidden;
                return;
            }
            var adminEmail = context.User.FindFirst(ClaimTypes.Email)?.Value ?? context.User.Identity?.Name;

            var socket = new ShellSocket(await context.WebSockets.AcceptWebSocketAsync());

            OpenResult result;
            try
            {
                result = await _relay.OpenSessionAsync(new OpenRequest(
                    robotId, robotName, adminUserId, adminEmail, socket,
                    ClampSize(query["cols"], 80), ClampSize(query["rows"], 24),
                    bool.TryParse(query["force"], out var force) && force));
            }
            catch (Exception)
            {
                await socket.SendJsonAsync(new AdminError("Could not start the session."));
                await socket.CloseAsync(WebSocketCloseStatus.InternalServerError, "error");
                return;
            }

            switch (result.Status)
            {
                case OpenStatus.Offline:
                    await RejectAsync(socket, new AdminError("Robot is offline."));
                    return;
                case OpenStatus.RateLimited:
                    await RejectAsync(socket, new AdminError("Too many sessions opened; wait a minute and try again."));
                    return;
                case OpenStatus.Busy:
                    await RejectAsync(socket, new AdminBusy(result.HeldBy, result.Since!.Value));
                    return;
            }

            var session = result.Session!;
            await socket.SendJsonAsync(new AdminOpened(session.Id, robotName));

            using var lifetime = CancellationTokenSource.CreateLinkedTokenSource(
                session.Lifetime.Token, context.RequestAborted);
            var closedByAdmin = false;
            try
            {
                closedByAdmin = await ReceiveLoopAsync(session, socket, lifetime.Token);
            }
            finally
            {
                // No-op if the session already ended (exited, taken over, robot dropped).
                await _relay.EndSessionAsync(session.Id,
                    closedByAdmin ? ShellEndReason.AdminClosed : ShellEndReason.AdminDisconnected);
            }
        }

        // Returns true if the admin asked to close, false if the socket just went away.
        private async Task<bool> ReceiveLoopAsync(ShellSession session, ShellSocket socket, CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                var message = await socket.ReceiveAsync(ct);
                if (message == null)
                    return false;

                var (type, data) = message.Value;
                if (type == WebSocketMessageType.Binary)
                {
                    await _relay.AdminInputAsync(session, data);
                    continue;
                }

                ShellMessage? envelope;
                try
                {
                    envelope = JsonSerializer.Deserialize<ShellMessage>(data, ShellProtocol.Json);
                }
                catch (JsonException)
                {
                    continue;
                }

                switch (envelope?.Type)
                {
                    case "resize":
                        var resize = JsonSerializer.Deserialize<AdminResize>(data, ShellProtocol.Json);
                        if (resize != null)
                            await _relay.AdminResizeAsync(session, Clamp(resize.Cols), Clamp(resize.Rows));
                        break;
                    case "close":
                        return true;
                }
            }
            return false;
        }

        private static async Task RejectAsync<T>(ShellSocket socket, T message)
        {
            await socket.SendJsonAsync(message);
            await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "rejected");
        }

        private bool IsAllowedOrigin(HttpContext context)
        {
            var origin = context.Request.Headers.Origin.ToString();
            if (string.IsNullOrEmpty(origin))
                return false;  // browsers always send Origin on WebSocket handshakes
            if (_options.Value.AllowedOrigins.Contains(origin, StringComparer.OrdinalIgnoreCase))
                return true;
            return Uri.TryCreate(origin, UriKind.Absolute, out var uri)
                && string.Equals(uri.Authority, context.Request.Host.Value, StringComparison.OrdinalIgnoreCase);
        }

        private static int ClampSize(string? value, int fallback) =>
            int.TryParse(value, out var n) ? Clamp(n) : fallback;

        private static int Clamp(int n) => Math.Clamp(n, 1, 1000);
    }
}
