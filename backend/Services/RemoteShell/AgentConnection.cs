// bloom
// AgentConnection.cs
// Handles one robot agent's tunnel on /ws/robot-agent for its whole lifetime. The endpoint's
// "RobotAgent" policy has already validated the JWT (signature, expiry, current TokenVersion);
// the robot's identity is taken from that token and nothing else.

using System.IdentityModel.Tokens.Jwt;
using System.Net.WebSockets;
using System.Text.Json;
using bloom.Data;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Options;
using Semver;

namespace bloom.Services.RemoteShell
{
    public class AgentConnection
    {
        private static readonly TimeSpan HelloTimeout = TimeSpan.FromSeconds(10);

        private readonly IShellRelay _relay;
        private readonly BloomDbContext _db;
        private readonly IOptions<RemoteShellOptions> _options;
        private readonly TimeProvider _time;
        private readonly ILogger<AgentConnection> _logger;

        public AgentConnection(
            IShellRelay relay,
            BloomDbContext db,
            IOptions<RemoteShellOptions> options,
            TimeProvider time,
            ILogger<AgentConnection> logger)
        {
            _relay = relay;
            _db = db;
            _options = options;
            _time = time;
            _logger = logger;
        }

        public async Task RunAsync(HttpContext context)
        {
            if (!context.WebSockets.IsWebSocketRequest)
            {
                context.Response.StatusCode = StatusCodes.Status400BadRequest;
                return;
            }
            if (!Guid.TryParse(context.User.FindFirst("robotId")?.Value, out var robotId))
            {
                context.Response.StatusCode = StatusCodes.Status403Forbidden;
                return;
            }

            var socket = new ShellSocket(await context.WebSockets.AcceptWebSocketAsync());

            var hello = await ReceiveHelloAsync(socket, context.RequestAborted);
            if (hello == null)
            {
                await socket.CloseAsync(WebSocketCloseStatus.ProtocolError, "expected hello");
                return;
            }

            var version = NormalizeVersion(robotId, hello.Version);
            var link = await _relay.RegisterAgentAsync(robotId, socket, version, hello.Commit);
            await RecordSeenAsync(robotId, version, hello.Commit);

            // The tunnel lives no longer than the token that opened it. The agent re-logs in and
            // reconnects, so a revoked robot can't hold a tunnel open past expiry either.
            var untilExpiry = TokenExpiry(context) is DateTimeOffset exp
                ? exp - _time.GetUtcNow()
                : Timeout.InfiniteTimeSpan;
            if (untilExpiry != Timeout.InfiniteTimeSpan && untilExpiry < TimeSpan.Zero)
                untilExpiry = TimeSpan.Zero;
            // At expiry, send the close frame (with "token_expired", so the agent reconnects at once
            // rather than backing off) and let the agent's close reply end the receive loop.
            // Cancelling the receive directly would make Kestrel abort before the frame goes out;
            // `fallback` does that only if the agent never answers.
            using var expiry = new CancellationTokenSource(untilExpiry, _time);
            using var fallback = new CancellationTokenSource();
            using var onExpiry = expiry.Token.Register(() =>
            {
                fallback.CancelAfter(TimeSpan.FromSeconds(5));
                _ = socket.CloseAsync(WebSocketCloseStatus.PolicyViolation, ShellEndReason.TokenExpired);
            });
            using var lifetime = CancellationTokenSource.CreateLinkedTokenSource(
                link.Lifetime.Token, context.RequestAborted, fallback.Token);

            try
            {
                await ReceiveLoopAsync(robotId, socket, lifetime.Token);
            }
            finally
            {
                var expired = expiry.IsCancellationRequested;
                var reason = link.DropReason
                    ?? (expired ? ShellEndReason.TokenExpired : ShellEndReason.RobotDisconnected);
                await _relay.UnregisterAgentAsync(link, reason);
                if (!expired)
                    await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, reason);
                await RecordSeenAsync(robotId, version: null, commit: null);
            }
        }

        private async Task ReceiveLoopAsync(Guid robotId, ShellSocket socket, CancellationToken ct)
        {
            var idleTimeout = TimeSpan.FromSeconds(_options.Value.AgentIdleTimeoutSeconds);

            while (!ct.IsCancellationRequested)
            {
                using var idle = CancellationTokenSource.CreateLinkedTokenSource(ct);
                idle.CancelAfter(idleTimeout);

                var message = await socket.ReceiveAsync(idle.Token);
                if (message == null)
                {
                    if (!ct.IsCancellationRequested && idle.IsCancellationRequested)
                        _logger.LogWarning("Robot {RobotId} agent silent for {Timeout}; dropping tunnel", robotId, idleTimeout);
                    return;
                }

                var (type, data) = message.Value;
                if (type == WebSocketMessageType.Binary)
                {
                    if (ShellProtocol.TryDecodeDataFrame(data, out var sessionId, out var payload))
                        await _relay.AgentOutputAsync(robotId, sessionId, payload);
                    continue;
                }

                await HandleControlAsync(robotId, socket, data);
            }
        }

        private async Task HandleControlAsync(Guid robotId, ShellSocket socket, byte[] data)
        {
            ShellMessage? envelope;
            try
            {
                envelope = JsonSerializer.Deserialize<ShellMessage>(data, ShellProtocol.Json);
            }
            catch (JsonException)
            {
                return;
            }

            switch (envelope?.Type)
            {
                case "ping":
                    await socket.SendJsonAsync(new { type = "pong" });
                    break;
                case "secret_applied":
                    var applied = JsonSerializer.Deserialize<AgentSecretApplied>(data, ShellProtocol.Json);
                    if (applied != null)
                        _relay.CompleteSecretPush(robotId, applied.RequestId, applied.Ok, applied.Message);
                    break;
                case "exited":
                    var exited = JsonSerializer.Deserialize<AgentExited>(data, ShellProtocol.Json);
                    if (exited != null)
                        await _relay.AgentExitedAsync(robotId, exited.SessionId);
                    break;
            }
        }

        private static async Task<AgentHello?> ReceiveHelloAsync(ShellSocket socket, CancellationToken ct)
        {
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(ct);
            timeout.CancelAfter(HelloTimeout);

            var message = await socket.ReceiveAsync(timeout.Token);
            if (message is not { Type: WebSocketMessageType.Text } m)
                return null;
            try
            {
                var hello = JsonSerializer.Deserialize<AgentHello>(m.Data, ShellProtocol.Json);
                return hello?.Type == "hello" ? hello : null;
            }
            catch (JsonException)
            {
                return null;
            }
        }

        // Versions are compared as SemVer from here on (fleet view now, OTA in Phase 2), so only
        // well-formed ones are stored.
        private string? NormalizeVersion(Guid robotId, string? version)
        {
            if (version != null && SemVersion.TryParse(version, SemVersionStyles.Strict, out var parsed))
                return parsed.ToString();

            _logger.LogWarning("Robot {RobotId} agent reported a non-SemVer version {Version}", robotId, version);
            return null;
        }

        private static DateTimeOffset? TokenExpiry(HttpContext context) =>
            long.TryParse(context.User.FindFirst(JwtRegisteredClaimNames.Exp)?.Value, out var exp)
                ? DateTimeOffset.FromUnixTimeSeconds(exp)
                : null;

        private async Task RecordSeenAsync(Guid robotId, string? version, string? commit)
        {
            try
            {
                var robot = await _db.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
                if (robot == null)
                    return;
                robot.LastSeenAt = _time.GetUtcNow().UtcDateTime;
                if (version != null)
                {
                    robot.AgentVersion = version;
                    robot.AgentCommit = commit;
                }
                await _db.SaveChangesAsync();
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to record agent status for robot {RobotId}", robotId);
            }
        }
    }
}
