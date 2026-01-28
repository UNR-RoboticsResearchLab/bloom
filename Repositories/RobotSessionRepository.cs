// bloom
// RobotSessionRepository.cs
// Repository for persisting RobotSession data and historical state snapshots to database
// Created: 11/18/2025

using bloom.Models;
using bloom.Data;
using Microsoft.EntityFrameworkCore;

namespace bloom.Repositories
{
    public class RobotSessionRepository : IRobotSessionRepository
    {
        private readonly BloomDbContext _dbContext;

        public RobotSessionRepository(BloomDbContext dbContext)
        {
            _dbContext = dbContext;
        }

        public async Task<RobotSession?> GetAsync(Guid id)
        {
            return await _dbContext.RobotSessions
                .Include(s => s.StateHistory)
                .FirstOrDefaultAsync(s => s.Id == id);
        }

        public async Task<IEnumerable<RobotSession>> GetAllAsync()
        {
            return await _dbContext.RobotSessions
                .OrderByDescending(s => s.CreatedAt)
                .ToListAsync();
        }

        public async Task AddAsync(RobotSession session)
        {
            if (session == null)
                throw new ArgumentNullException(nameof(session));

            _dbContext.RobotSessions.Add(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task AddStateHistoryAsync(Guid sessionId, RobotState state)
        {
            if (state == null)
                throw new ArgumentNullException(nameof(state));

            var session = await _dbContext.RobotSessions.FirstOrDefaultAsync(s => s.Id == sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var history = new RobotStateHistory
            {
                RobotSessionId = sessionId,
                RobotState = state,
                Timestamp = DateTime.UtcNow
            };

            _dbContext.RobotStateHistorys.Add(history);
            session.LastUpdatedAt = DateTime.UtcNow;
            await _dbContext.SaveChangesAsync();
        }

        public async Task<IEnumerable<RobotStateHistory>> GetHistoryAsync(Guid sessionId)
        {
            var session = await _dbContext.RobotSessions.FirstOrDefaultAsync(s => s.Id == sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            return await _dbContext.RobotStateHistorys
                .Where(h => h.RobotSessionId == sessionId)
                .OrderByDescending(h => h.Timestamp)
                .ToListAsync();
        }
    }
}
