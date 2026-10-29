// bloom
// ShellRelay.cs
// Process-wide remote shell state: which robots have a live agent connection, and which admin holds
// the (single) shell session on each robot. Registered as a Singleton, like
// InMemoryRobotStateRepository -- one backend instance only; scaling out would need a SignalR
// backplane (e.g. Redis) plus moving this state out of memory.
//
// The relay never trusts ids from message bodies: robot ids come from the agent's validated JWT
// (RobotAgentHub), sessions are looked up by connection / id and checked against the sender.

using System.Collections.Concurrent;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Options;

namespace bloom.Services.RemoteShell
{
    public sealed class AgentLink
    {
        public required Guid RobotId { get; init; }
        public required string ConnectionId { get; init; }
        public required HubCallerContext Context { get; init; }
        public string? Version { get; init; }
        public string? Commit { get; init; }
        public DateTime ConnectedAt { get; init; }

        // Cancelled when the link ends; stops its token-expiry timer.
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
        public required string AdminConnectionId { get; init; }
        public required HubCallerContext AdminContext { get; init; }
    }

    public enum OpenStatus { Opened, Offline, Busy, RateLimited, AlreadyOpen }

    public record OpenRequest(
        Guid RobotId, string RobotName, string AdminUserId, string? AdminEmail,
        HubCallerContext AdminContext, int Cols, int Rows, bool Force);

    public record OpenResult(OpenStatus Status, ShellSession? Session = null, string? HeldBy = null, DateTime? Since = null);

    public enum SecretPushStatus { Applied, Offline, Busy, Rejected, TimedOut, ConnectionLost }

    // Message is the agent's own explanation on Rejected; shown to the admin, so it is truncated.
    public record SecretPushResult(SecretPushStatus Status, string? Message = null);

    public record ActiveSessionInfo(Guid SessionId, string AdminUserId, string? AdminEmail, DateTime StartedAt);

    public interface IShellRelay
    {
        Task<AgentLink> RegisterAgentAsync(Guid robotId, HubCallerContext context, string? version, string? commit, DateTimeOffset? tokenExpiresAt);
        Task UnregisterAgentAsync(AgentLink link, string reason);
        bool IsOnline(Guid robotId);
        AgentLink? GetAgent(Guid robotId);
        ActiveSessionInfo? GetActiveSession(Guid robotId);

        Task<OpenResult> OpenSessionAsync(OpenRequest request);
        Task AdminInputAsync(string adminConnectionId, byte[] data);
        Task AdminResizeAsync(string adminConnectionId, int cols, int rows);
        Task EndAdminSessionAsync(string adminConnectionId, string reason);
        Task AgentOutputAsync(Guid robotId, Guid sessionId, byte[] data);
        Task AgentExitedAsync(Guid robotId, Guid sessionId);
        Task EndSessionAsync(Guid sessionId, string reason, bool notifyAgent = true);

        /// <summary>
        /// Sends a new secret to the robot's agent over its live connection and waits for it to
        /// confirm it has applied it. Applied means the robot now holds the secret; TimedOut and
        /// ConnectionLost mean it is unknown whether it does.
        /// </summary>
        Task<SecretPushResult> PushSecretAsync(Guid robotId, string secret, TimeSpan timeout);
        void CompleteSecretPush(Guid robotId, Guid requestId, bool ok, string? message);

        /// <summary>Drops a robot's connection and ends its session, e.g. on revoke.</summary>
        Task DisconnectRobotAsync(Guid robotId, string reason);
    }

    public class ShellRelay : IShellRelay
    {
        private readonly IHubContext<RobotAgentHub, IAgentClient> _agentHub;
        private readonly IHubContext<ShellHub, IAdminClient> _adminHub;
        private readonly IServiceScopeFactory _scopeFactory;
        private readonly IOptions<RemoteShellOptions> _options;
        private readonly TimeProvider _time;
        private readonly ILogger<ShellRelay> _logger;

        // A hub can't close a connection with a reason, so a peer is told why and given a moment to
        // hang up cleanly before the server aborts it.
        private static readonly TimeSpan PeerCloseGrace = TimeSpan.FromSeconds(5);

        private readonly ConcurrentDictionary<Guid, AgentLink> _agents = new();
        private readonly ConcurrentDictionary<Guid, ShellSession> _sessions = new();
        private readonly ConcurrentDictionary<string, ShellSession> _sessionByAdminConnection = new();

        private sealed record PendingPush(Guid RobotId, TaskCompletionSource<SecretPushResult> Completion);
        private readonly ConcurrentDictionary<Guid, PendingPush> _pushes = new();  // by request id

        // Guarded by _gate: the robot -> session lock and the per-admin open history, which both
        // need check-then-act.
        private readonly object _gate = new();
        private readonly Dictionary<Guid, Guid> _sessionByRobot = new();
        private readonly Dictionary<string, Queue<DateTimeOffset>> _opensByAdmin = new();

        public ShellRelay(
            IHubContext<RobotAgentHub, IAgentClient> agentHub,
            IHubContext<ShellHub, IAdminClient> adminHub,
            IServiceScopeFactory scopeFactory,
            IOptions<RemoteShellOptions> options,
            TimeProvider time,
            ILogger<ShellRelay> logger)
        {
            _agentHub = agentHub;
            _adminHub = adminHub;
            _scopeFactory = scopeFactory;
            _options = options;
            _time = time;
            _logger = logger;
        }

        // ── Agents ──────────────────────────────────────────────────────────────

        public async Task<AgentLink> RegisterAgentAsync(
            Guid robotId, HubCallerContext context, string? version, string? commit, DateTimeOffset? tokenExpiresAt)
        {
            var link = new AgentLink
            {
                RobotId = robotId,
                ConnectionId = context.ConnectionId,
                Context = context,
                Version = version,
                Commit = commit,
                ConnectedAt = _time.GetUtcNow().UtcDateTime,
            };

            AgentLink? previous = null;
            _agents.AddOrUpdate(robotId, link, (_, existing) => { previous = existing; return link; });

            if (previous != null)
            {
                // A reconnect means the old connection is dead, and the agent kills its PTYs when a
                // connection drops, so any session on it is gone too.
                _logger.LogInformation("Robot {RobotId} agent reconnected; replacing stale link", robotId);
                await EndRobotSessionAsync(robotId, ShellEndReason.RobotDisconnected, notifyAgent: false);
                FailPushes(robotId);
                previous.DropReason = "replaced";
                await HangUpAsync(previous, "replaced");
            }

            // The connection lives no longer than the token that opened it. The agent re-logs in
            // and reconnects, so a revoked robot can't hold a connection past expiry either.
            if (tokenExpiresAt is { } exp)
                _ = ExpireAtAsync(link, exp);

            _logger.LogInformation("Robot {RobotId} agent online (version {Version}, commit {Commit})", robotId, version, commit);
            return link;
        }

        public async Task UnregisterAgentAsync(AgentLink link, string reason)
        {
            link.Lifetime.Cancel();

            // Only remove the link if it's still the current one -- a replaced link must not
            // tear down its replacement.
            if (!_agents.TryRemove(new KeyValuePair<Guid, AgentLink>(link.RobotId, link)))
                return;

            _logger.LogInformation("Robot {RobotId} agent offline ({Reason})", link.RobotId, reason);
            FailPushes(link.RobotId);
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

        private async Task ExpireAtAsync(AgentLink link, DateTimeOffset expiresAt)
        {
            try
            {
                var remaining = expiresAt - _time.GetUtcNow();
                if (remaining > TimeSpan.Zero)
                    await Task.Delay(remaining, _time, link.Lifetime.Token);
            }
            catch (OperationCanceledException)
            {
                return;  // the link ended first
            }

            if (!_agents.TryRemove(new KeyValuePair<Guid, AgentLink>(link.RobotId, link)))
                return;
            _logger.LogInformation("Robot {RobotId} agent token expired; dropping connection", link.RobotId);
            link.DropReason = ShellEndReason.TokenExpired;
            FailPushes(link.RobotId);
            await EndRobotSessionAsync(link.RobotId, ShellEndReason.TokenExpired, notifyAgent: false);
            await HangUpAsync(link, ShellEndReason.TokenExpired);
        }

        private async Task HangUpAsync(AgentLink link, string reason)
        {
            try
            {
                await _agentHub.Clients.Client(link.ConnectionId).Disconnecting(reason);
            }
            catch (Exception ex)
            {
                _logger.LogDebug(ex, "Couldn't tell robot {RobotId} why it is being dropped", link.RobotId);
            }
            AbortLater(link.Context);
        }

        private static void AbortLater(HubCallerContext context) =>
            _ = Task.Delay(PeerCloseGrace).ContinueWith(_ => context.Abort(), TaskScheduler.Default);

        // ── Sessions ────────────────────────────────────────────────────────────

        public async Task<OpenResult> OpenSessionAsync(OpenRequest request)
        {
            ShellSession session;
            Guid? takenOver = null;

            lock (_gate)
            {
                // One connection, one session: a second attempt (e.g. "take over") uses a new connection.
                if (_sessionByAdminConnection.ContainsKey(request.AdminContext.ConnectionId))
                    return new OpenResult(OpenStatus.AlreadyOpen);

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
                    AdminConnectionId = request.AdminContext.ConnectionId,
                    AdminContext = request.AdminContext,
                };
                _sessions[session.Id] = session;
                _sessionByRobot[request.RobotId] = session.Id;
                _sessionByAdminConnection[session.AdminConnectionId] = session;
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
                await _agentHub.Clients.Client(agent.ConnectionId).Open(session.Id, request.Cols, request.Rows);

            return new OpenResult(OpenStatus.Opened, session);
        }

        public async Task AdminInputAsync(string adminConnectionId, byte[] data)
        {
            if (!_sessionByAdminConnection.TryGetValue(adminConnectionId, out var session)
                || GetAgent(session.RobotId) is not { } agent)
                return;
            await _agentHub.Clients.Client(agent.ConnectionId).Input(session.Id, data);
        }

        public async Task AdminResizeAsync(string adminConnectionId, int cols, int rows)
        {
            if (!_sessionByAdminConnection.TryGetValue(adminConnectionId, out var session)
                || GetAgent(session.RobotId) is not { } agent)
                return;
            await _agentHub.Clients.Client(agent.ConnectionId).Resize(session.Id, cols, rows);
        }

        public async Task EndAdminSessionAsync(string adminConnectionId, string reason)
        {
            if (_sessionByAdminConnection.TryGetValue(adminConnectionId, out var session))
                await EndSessionAsync(session.Id, reason);
        }

        public async Task AgentOutputAsync(Guid robotId, Guid sessionId, byte[] data)
        {
            // A robot may only write into sessions on itself.
            if (!_sessions.TryGetValue(sessionId, out var session) || session.RobotId != robotId)
                return;
            await _adminHub.Clients.Client(session.AdminConnectionId).Output(data);
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
                await _agentHub.Clients.Client(agent.ConnectionId).Close(sessionId);

            try
            {
                await _adminHub.Clients.Client(session.AdminConnectionId).SessionClosed(reason);
            }
            catch (Exception ex)
            {
                _logger.LogDebug(ex, "Admin connection for session {SessionId} was already gone", sessionId);
            }
            // The browser hangs up on SessionClosed; this is the fallback for one that doesn't.
            AbortLater(session.AdminContext);

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

        // ── Credential push ─────────────────────────────────────────────────────

        public async Task<SecretPushResult> PushSecretAsync(Guid robotId, string secret, TimeSpan timeout)
        {
            if (GetAgent(robotId) is not { } agent)
                return new SecretPushResult(SecretPushStatus.Offline);

            var requestId = Guid.NewGuid();
            var pending = new PendingPush(robotId, new(TaskCreationOptions.RunContinuationsAsynchronously));
            lock (_gate)
            {
                if (_pushes.Values.Any(p => p.RobotId == robotId))
                    return new SecretPushResult(SecretPushStatus.Busy);
                _pushes[requestId] = pending;
            }

            try
            {
                await _agentHub.Clients.Client(agent.ConnectionId).SetSecret(requestId, secret);
                using var cts = new CancellationTokenSource(timeout, _time);
                using var _ = cts.Token.Register(
                    () => pending.Completion.TrySetResult(new SecretPushResult(SecretPushStatus.TimedOut)));
                return await pending.Completion.Task;
            }
            finally
            {
                _pushes.TryRemove(requestId, out var removed);
            }
        }

        public void CompleteSecretPush(Guid robotId, Guid requestId, bool ok, string? message)
        {
            // An agent may only answer pushes sent to itself.
            if (!_pushes.TryGetValue(requestId, out var pending) || pending.RobotId != robotId)
                return;

            var reason = string.IsNullOrWhiteSpace(message) ? null : message.Trim();
            if (reason is { Length: > 200 })
                reason = reason[..200];
            pending.Completion.TrySetResult(ok
                ? new SecretPushResult(SecretPushStatus.Applied)
                : new SecretPushResult(SecretPushStatus.Rejected, reason));
        }

        private void FailPushes(Guid robotId)
        {
            foreach (var pending in _pushes.Values.Where(p => p.RobotId == robotId))
                pending.Completion.TrySetResult(new SecretPushResult(SecretPushStatus.ConnectionLost));
        }

        // ── Dropping robots ─────────────────────────────────────────────────────

        public async Task DisconnectRobotAsync(Guid robotId, string reason)
        {
            await EndRobotSessionAsync(robotId, reason, notifyAgent: false);
            FailPushes(robotId);
            if (_agents.TryRemove(robotId, out var link))
            {
                _logger.LogWarning("Robot {RobotId} connection dropped ({Reason})", robotId, reason);
                link.DropReason = reason;
                link.Lifetime.Cancel();
                await HangUpAsync(link, reason);
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

        // Returns false if the session was already removed (i.e. someone else is ending it).
        private bool RemoveSession(ShellSession session)
        {
            lock (_gate)
            {
                if (!_sessions.TryRemove(session.Id, out _))
                    return false;
                if (_sessionByRobot.TryGetValue(session.RobotId, out var id) && id == session.Id)
                    _sessionByRobot.Remove(session.RobotId);
                _sessionByAdminConnection.TryRemove(new KeyValuePair<string, ShellSession>(session.AdminConnectionId, session));
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
