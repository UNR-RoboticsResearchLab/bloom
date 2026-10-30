// bloom
// ShellHub.cs
// The admin's browser terminal, at /hubs/shell (policy "AdminOnly": the Identity cookie, Admin role;
// robot JWTs never satisfy it). One connection carries at most one shell session: call OpenSession,
// then Input / Resize / CloseSession, and receive Output / SessionClosed (IAdminClient). A new attempt
// (e.g. "take over") uses a new connection.

using System.Security.Claims;
using bloom.Data;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services.RemoteShell
{
    [Authorize(Policy = "AdminOnly")]
    public class ShellHub : Hub<IAdminClient>
    {
        private readonly IShellRelay _relay;
        private readonly BloomDbContext _db;

        public ShellHub(IShellRelay relay, BloomDbContext db)
        {
            _relay = relay;
            _db = db;
        }

        public async Task<OpenSessionResult> OpenSession(Guid robotId, int cols, int rows, bool force)
        {
            var robotName = await _db.Robots.Where(r => r.Id == robotId).Select(r => r.Name).FirstOrDefaultAsync();
            if (robotName == null)
                return new OpenSessionResult("not_found", Message: "Robot not found.");

            var adminUserId = Context.User?.FindFirst(ClaimTypes.NameIdentifier)?.Value;
            if (adminUserId == null)
                throw new HubException("Not signed in.");
            var adminEmail = Context.User?.FindFirst(ClaimTypes.Email)?.Value ?? Context.User?.Identity?.Name;

            var result = await _relay.OpenSessionAsync(new OpenRequest(
                robotId, robotName, adminUserId, adminEmail, Context, Clamp(cols, 80), Clamp(rows, 24), force));

            return result.Status switch
            {
                OpenStatus.Opened => new OpenSessionResult("opened", result.Session!.Id, robotName),
                OpenStatus.Offline => new OpenSessionResult("offline", Message: "Robot is offline."),
                OpenStatus.RateLimited => new OpenSessionResult("rate_limited", Message: "Too many sessions opened; wait a minute and try again."),
                OpenStatus.AlreadyOpen => new OpenSessionResult("already_open", Message: "This connection already has a session."),
                _ => new OpenSessionResult("busy", HeldBy: result.HeldBy, Since: result.Since),
            };
        }

        public Task Input(byte[] data) => _relay.AdminInputAsync(Context.ConnectionId, data);

        public Task Resize(int cols, int rows) =>
            _relay.AdminResizeAsync(Context.ConnectionId, Clamp(cols, 80), Clamp(rows, 24));

        public Task CloseSession() => _relay.EndAdminSessionAsync(Context.ConnectionId, ShellEndReason.AdminClosed);

        public override async Task OnDisconnectedAsync(Exception? exception)
        {
            // No-op if the session already ended (exited, taken over, robot dropped).
            await _relay.EndAdminSessionAsync(Context.ConnectionId, ShellEndReason.AdminDisconnected);
            await base.OnDisconnectedAsync(exception);
        }

        private static int Clamp(int n, int fallback) => n <= 0 ? fallback : Math.Min(n, 1000);
    }
}
