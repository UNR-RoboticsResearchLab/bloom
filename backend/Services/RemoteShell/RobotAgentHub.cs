// bloom
// RobotAgentHub.cs
// The robot agent's side of the remote shell, at /hubs/robot-agent (WebSockets only, policy
// "RobotAgent": a JWT from a RobotId + Secret login with a current TokenVersion). The robot's
// identity is the validated token's robotId claim and nothing else -- no method takes a robot id.
//
// Agent -> server: Hello, Output, SessionExited, SecretApplied.
// Server -> agent: IAgentClient (Open, Input, Resize, Close, SetSecret, Disconnecting).

using System.IdentityModel.Tokens.Jwt;
using bloom.Data;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Semver;

namespace bloom.Services.RemoteShell
{
    [Authorize(Policy = "RobotAgent")]
    public class RobotAgentHub : Hub<IAgentClient>
    {
        private const string LinkKey = "agent-link";
        private static readonly TimeSpan HelloTimeout = TimeSpan.FromSeconds(10);

        private readonly IShellRelay _relay;
        private readonly BloomDbContext _db;
        private readonly TimeProvider _time;
        private readonly ILogger<RobotAgentHub> _logger;

        public RobotAgentHub(IShellRelay relay, BloomDbContext db, TimeProvider time, ILogger<RobotAgentHub> logger)
        {
            _relay = relay;
            _db = db;
            _time = time;
            _logger = logger;
        }

        private Guid RobotId =>
            Guid.TryParse(Context.User?.FindFirst("robotId")?.Value, out var id) ? id : Guid.Empty;

        public override Task OnConnectedAsync()
        {
            if (RobotId == Guid.Empty)
            {
                Context.Abort();
                return Task.CompletedTask;
            }

            // An agent that connects but never says hello is not an agent.
            var context = Context;
            _ = Task.Delay(HelloTimeout, _time).ContinueWith(
                _ => { if (!context.Items.ContainsKey(LinkKey)) context.Abort(); },
                TaskScheduler.Default);
            return base.OnConnectedAsync();
        }

        /// <summary>First call on every connection: reports what the agent is running.</summary>
        public async Task Hello(string? version, string? commit)
        {
            if (Context.Items.ContainsKey(LinkKey))
                return;

            var normalized = NormalizeVersion(version);
            DateTimeOffset? expiresAt = long.TryParse(Context.User?.FindFirst(JwtRegisteredClaimNames.Exp)?.Value, out var exp)
                ? DateTimeOffset.FromUnixTimeSeconds(exp)
                : null;

            var link = await _relay.RegisterAgentAsync(RobotId, Context, normalized, commit, expiresAt);
            Context.Items[LinkKey] = link;
            await RecordSeenAsync(RobotId, normalized, commit);
        }

        public Task Output(Guid sessionId, byte[] data) => _relay.AgentOutputAsync(RobotId, sessionId, data);

        public Task SessionExited(Guid sessionId, int? code) => _relay.AgentExitedAsync(RobotId, sessionId);

        public void SecretApplied(Guid requestId, bool ok, string? message) =>
            _relay.CompleteSecretPush(RobotId, requestId, ok, message);

        public override async Task OnDisconnectedAsync(Exception? exception)
        {
            if (Context.Items.TryGetValue(LinkKey, out var value) && value is AgentLink link)
            {
                await _relay.UnregisterAgentAsync(link, link.DropReason ?? ShellEndReason.RobotDisconnected);
                await RecordSeenAsync(link.RobotId, version: null, commit: null);
            }
            await base.OnDisconnectedAsync(exception);
        }

        // Versions are compared as SemVer from here on (fleet view now, OTA in Phase 2), so only
        // well-formed ones are stored.
        private string? NormalizeVersion(string? version)
        {
            if (version != null && SemVersion.TryParse(version, SemVersionStyles.Strict, out var parsed))
                return parsed.ToString();

            _logger.LogWarning("Robot {RobotId} agent reported a non-SemVer version {Version}", RobotId, version);
            return null;
        }

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
