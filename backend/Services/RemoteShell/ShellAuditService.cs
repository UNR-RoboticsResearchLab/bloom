// bloom
// ShellAuditService.cs
// Persists ShellSessionAudit rows. Scoped (it holds a BloomDbContext); the singleton ShellRelay
// creates a fresh scope per call rather than capturing one.

using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services.RemoteShell
{
    public interface IShellAuditService
    {
        Task RecordStartAsync(ShellSession session);
        Task RecordEndAsync(Guid sessionId, string reason, DateTime endedAt);
        Task<IReadOnlyList<ShellSessionAudit>> GetForRobotAsync(Guid robotId, int take = 50);
    }

    public class ShellAuditService : IShellAuditService
    {
        private readonly BloomDbContext _db;

        public ShellAuditService(BloomDbContext db)
        {
            _db = db;
        }

        public async Task RecordStartAsync(ShellSession session)
        {
            _db.ShellSessionAudits.Add(new ShellSessionAudit
            {
                Id = session.Id,
                RobotId = session.RobotId,
                AdminUserId = session.AdminUserId,
                AdminEmail = session.AdminEmail,
                StartedAt = session.StartedAt,
            });
            await _db.SaveChangesAsync();
        }

        public async Task RecordEndAsync(Guid sessionId, string reason, DateTime endedAt)
        {
            var audit = await _db.ShellSessionAudits.FirstOrDefaultAsync(a => a.Id == sessionId);
            if (audit == null || audit.EndedAt != null)
                return;

            audit.EndedAt = endedAt;
            audit.EndReason = reason;
            await _db.SaveChangesAsync();
        }

        public async Task<IReadOnlyList<ShellSessionAudit>> GetForRobotAsync(Guid robotId, int take = 50)
        {
            return await _db.ShellSessionAudits
                .Where(a => a.RobotId == robotId)
                .OrderByDescending(a => a.StartedAt)
                .Take(take)
                .ToListAsync();
        }
    }
}
