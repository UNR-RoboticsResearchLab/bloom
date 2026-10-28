// bloom
// ShellRelay.cs
// Process-wide remote shell state: which robots have a live agent tunnel, and which admin holds
// the (single) shell session on each robot. Registered as a Singleton, like
// InMemoryRobotStateRepository -- one backend instance only; scaling out would need a backplane.
//
// The relay never trusts ids from message bodies: robot ids come from the agent's validated JWT
// (AgentConnection), sessions are looked up by id and checked against the sender's robot/admin.

using System.Collections.Concurrent;
using System.Net.WebSockets;
using Microsoft.Extensions.Options;

namespace bloom.Services.RemoteShell
{
    public sealed class AgentLink
    {
        public required Guid RobotId { get; init; }
        public required ShellSocket Socket { get; init; }
        public string? Version { get; init; }
        public string? Commit { get; init; }
        public DateTime ConnectedAt { get; init; }

        // Cancelling this ends the agent's receive loop; DropReason says why.
        public CancellationTokenSource Lifetime { get; } = new();
        public string? DropReason { get; set; }
    }

    public sealed class ShellSession
    {
        public required Guid Id { get; init; }
        public required Guid RobotId { get; init; }
        public required string RobotName { get; init; }
        public required string AdminUserId { get; init; }
        public string? AdminEmail { get; init; }
        public DateTime StartedAt { get; init; }
        public required ShellSocket AdminSocket { get; init; }

        // Cancelling this ends the admin's receive loop.
        public CancellationTokenSource Lifetime { get; } = new();
    }

    public enum OpenStatus { Opened, Offline, Busy, RateLimited }

    public record OpenRequest(
        Guid RobotId, string RobotName, string AdminUserId, string? AdminEmail,
        ShellSocket AdminSocket, int Cols, int Rows, bool Force);

    public record OpenResult(OpenStatus Status, ShellSession? Session = null, string? HeldBy = null, DateTime? Since = null);

    public record ActiveSessionInfo(Guid SessionId, string AdminUserId, string? AdminEmail, DateTime StartedAt);

    public interface IShellRelay
    {
        Task<AgentLink> RegisterAgentAsync(Guid robotId, ShellSocket socket, string? version, string? commit);
        Task UnregisterAgentAsync(AgentLink link, string reason);
        bool IsOnline(Guid robotId);
        AgentLink? GetAgent(Guid robotId);
        ActiveSessionInfo? GetActiveSession(Guid robotId);

        Task<OpenResult> OpenSessionAsync(OpenRequest request);
        Task AdminInputAsync(ShellSession session, ReadOnlyMemory<byte> data);
        Task AdminResizeAsync(ShellSession session, int cols, int rows);
        Task AgentOutputAsync(Guid robotId, Guid sessionId, ReadOnlyMemory<byte> data);
        Task AgentExitedAsync(Guid robotId, Guid sessionId);
        Task EndSessionAsync(Guid sessionId, string reason, bool notifyAgent = true);

        /// <summary>Drops a robot's tunnel and ends its session, e.g. on revoke.</summary>
        Task DisconnectRobotAsync(Guid robotId, string reason);
    }

    public class ShellRelay : IShellRelay
    {
        private readonly IServiceScopeFactory _scopeFactory;
        private readonly IOptions<RemoteShellOptions> _options;
        private readonly TimeProvider _time;
        private readonly ILogger<ShellRelay> _logger;

        private readonly ConcurrentDictionary<Guid, AgentLink> _agents = new();
        private readonly ConcurrentDictionary<Guid, ShellSession> _sessions = new();

        // Guarded by _gate: the robot -> session lock and the per-admin open history, which both
        // need check-then-act.
        private readonly object _gate = new();
        private readonly Dictionary<Guid, Guid> _sessionByRobot = new();
        private readonly Dictionary<string, Queue<DateTimeOffset>> _opensByAdmin = new();

        public ShellRelay(
            IServiceScopeFactory scopeFactory,
            IOptions<RemoteShellOptions> options,
            TimeProvider time,
            ILogger<ShellRelay> logger)
        {
            _scopeFactory = scopeFactory;
            _options = options;
            _time = time;
            _logger = logger;
        }

        public async Task<AgentLink> RegisterAgentAsync(Guid robotId, ShellSocket socket, string? version, string? commit)
        {
            var link = new AgentLink
            {
                RobotId = robotId,
                Socket = socket,
                Version = version,
                Commit = commit,
                ConnectedAt = _time.GetUtcNow().UtcDateTime,
            };

            AgentLink? previous = null;
            _agents.AddOrUpdate(robotId, link, (_, existing) => { previous = existing; return link; });

            if (previous != null)
            {
                // A reconnect means the old tunnel is dead, and the agent kills its PTYs when a
                // tunnel drops, so any session on it is gone too.
                _logger.LogInformation("Robot {RobotId} agent reconnected; replacing stale link", robotId);
                await EndRobotSessionAsync(robotId, ShellEndReason.RobotDisconnected, notifyAgent: false);
                previous.DropReason = "replaced";
                previous.Lifetime.Cancel();
            }

            _logger.LogInformation("Robot {RobotId} agent online (version {Version}, commit {Commit})", robotId, version, commit);
            return link;
        }

        public async Task UnregisterAgentAsync(AgentLink link, string reason)
        {
            // Only remove the link if it's still the current one -- a replaced link must not
            // tear down its replacement.
            if (!_agents.TryRemove(new KeyValuePair<Guid, AgentLink>(link.RobotId, link)))
                return;

            _logger.LogInformation("Robot {RobotId} agent offline ({Reason})", link.RobotId, reason);
            await EndRobotSessionAsync(link.RobotId, reason, notifyAgent: false);
        }

        public bool IsOnline(Guid robotId) => _agents.ContainsKey(robotId);

        public AgentLink? GetAgent(Guid robotId) => _agents.TryGetValue(robotId, out var link) ? link : null;

        public ActiveSessionInfo? GetActiveSession(Guid robotId)
        {
            lock (_gate)
            {
                if (_sessionByRobot.TryGetValue(robotId, out var id) && _sessions.TryGetValue(id, out var s))
                    return new ActiveSessionInfo(s.Id, s.AdminUserId, s.AdminEmail, s.StartedAt);
            }
            return null;
        }

        public async Task<OpenResult> OpenSessionAsync(OpenRequest request)
        {
            ShellSession session;
            Guid? takenOver = null;

            lock (_gate)
            {
                if (!TryConsumeOpen(request.AdminUserId))
                    return new OpenResult(OpenStatus.RateLimited);

                if (!_agents.ContainsKey(request.RobotId))
                    return new OpenResult(OpenStatus.Offline);

                if (_sessionByRobot.TryGetValue(request.RobotId, out var existingId)
                    && _sessions.TryGetValue(existingId, out var existing))
                {
                    if (!request.Force)
                        return new OpenResult(OpenStatus.Busy, HeldBy: existing.AdminEmail ?? existing.AdminUserId, Since: existing.StartedAt);
                    takenOver = existingId;
                }

                session = new ShellSession
                {
                    Id = Guid.NewGuid(),
                    RobotId = request.RobotId,
                    RobotName = request.RobotName,
                    AdminUserId = request.AdminUserId,
                    AdminEmail = request.AdminEmail,
                    StartedAt = _time.GetUtcNow().UtcDateTime,
                    AdminSocket = request.AdminSocket,
                };
                _sessions[session.Id] = session;
                _sessionByRobot[request.RobotId] = session.Id;
            }

            if (takenOver is Guid oldId)
                await EndSessionAsync(oldId, ShellEndReason.TakenOver);

            try
            {
                await using var scope = _scopeFactory.CreateAsyncScope();
                await scope.ServiceProvider.GetRequiredService<IShellAuditService>().RecordStartAsync(session);
            }
            catch (Exception ex)
            {
                // No audit row, no shell.
                _logger.LogError(ex, "Failed to record shell session start; refusing session");
                RemoveSession(session);
                throw;
            }

            _logger.LogWarning("Shell session {SessionId} opened on robot {RobotId} by {Admin}",
                session.Id, session.RobotId, session.AdminEmail ?? session.AdminUserId);

            if (GetAgent(request.RobotId) is { } agent)
                await agent.Socket.SendJsonAsync(new AgentOpen(session.Id, request.Cols, request.Rows));

            return new OpenResult(OpenStatus.Opened, session);
        }

        public async Task AdminInputAsync(ShellSession session, ReadOnlyMemory<byte> data)
        {
            if (!IsCurrent(session) || GetAgent(session.RobotId) is not { } agent)
                return;
            await agent.Socket.SendBinaryAsync(ShellProtocol.EncodeDataFrame(session.Id, data.Span));
        }

        public async Task AdminResizeAsync(ShellSession session, int cols, int rows)
        {
            if (!IsCurrent(session) || GetAgent(session.RobotId) is not { } agent)
                return;
            await agent.Socket.SendJsonAsync(new AgentResize(session.Id, cols, rows));
        }

        public async Task AgentOutputAsync(Guid robotId, Guid sessionId, ReadOnlyMemory<byte> data)
        {
            // A robot may only write into sessions on itself.
            if (!_sessions.TryGetValue(sessionId, out var session) || session.RobotId != robotId)
                return;
            await session.AdminSocket.SendBinaryAsync(data);
        }

        public async Task AgentExitedAsync(Guid robotId, Guid sessionId)
        {
            if (!_sessions.TryGetValue(sessionId, out var session) || session.RobotId != robotId)
                return;
            await EndSessionAsync(sessionId, ShellEndReason.Exited, notifyAgent: false);
        }

        public async Task EndSessionAsync(Guid sessionId, string reason, bool notifyAgent = true)
        {
            if (!_sessions.TryGetValue(sessionId, out var session) || !RemoveSession(session))
                return;  // already ended

            _logger.LogWarning("Shell session {SessionId} on robot {RobotId} ended ({Reason})", sessionId, session.RobotId, reason);

            if (notifyAgent && GetAgent(session.RobotId) is { } agent)
                await agent.Socket.SendJsonAsync(new AgentClose(sessionId));

            await session.AdminSocket.SendJsonAsync(new AdminClosed(reason));
            await session.AdminSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, reason);
            session.Lifetime.Cancel();

            try
            {
                await using var scope = _scopeFactory.CreateAsyncScope();
                await scope.ServiceProvider.GetRequiredService<IShellAuditService>()
                    .RecordEndAsync(sessionId, reason, _time.GetUtcNow().UtcDateTime);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to record end of shell session {SessionId}", sessionId);
            }
        }

        public async Task DisconnectRobotAsync(Guid robotId, string reason)
        {
            await EndRobotSessionAsync(robotId, reason, notifyAgent: false);
            if (_agents.TryRemove(robotId, out var link))
            {
                _logger.LogWarning("Robot {RobotId} tunnel dropped ({Reason})", robotId, reason);
                link.DropReason = reason;
                await link.Socket.CloseAsync(WebSocketCloseStatus.PolicyViolation, reason);
                link.Lifetime.Cancel();
            }
        }

        private async Task EndRobotSessionAsync(Guid robotId, string reason, bool notifyAgent)
        {
            Guid? sessionId = null;
            lock (_gate)
            {
                if (_sessionByRobot.TryGetValue(robotId, out var id))
                    sessionId = id;
            }
            if (sessionId is Guid sid)
                await EndSessionAsync(sid, reason, notifyAgent);
        }

        private bool IsCurrent(ShellSession session) =>
            _sessions.TryGetValue(session.Id, out var current) && ReferenceEquals(current, session);

        // Returns false if the session was already removed (i.e. someone else is ending it).
        private bool RemoveSession(ShellSession session)
        {
            lock (_gate)
            {
                if (!_sessions.TryRemove(session.Id, out _))
                    return false;
                if (_sessionByRobot.TryGetValue(session.RobotId, out var id) && id == session.Id)
                    _sessionByRobot.Remove(session.RobotId);
                return true;
            }
        }

        // Caller holds _gate. Sliding one-minute window; every attempt counts, successful or not.
        private bool TryConsumeOpen(string adminUserId)
        {
            var now = _time.GetUtcNow();
            if (!_opensByAdmin.TryGetValue(adminUserId, out var opens))
                _opensByAdmin[adminUserId] = opens = new Queue<DateTimeOffset>();

            while (opens.Count > 0 && now - opens.Peek() >= TimeSpan.FromMinutes(1))
                opens.Dequeue();

            if (opens.Count >= _options.Value.SessionOpensPerMinute)
                return false;

            opens.Enqueue(now);
            return true;
        }
    }
}
