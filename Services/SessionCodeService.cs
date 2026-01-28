// bloom
// SessionCodeService.cs
// Service for generating and managing 6-digit session codes
// Created: 11/21/2025

using bloom.Data;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class SessionCodeService : ISessionCodeService
    {
        private readonly BloomDbContext _dbContext;
        private readonly Random _random;

        public SessionCodeService(BloomDbContext dbContext)
        {
            _dbContext = dbContext;
            _random = new Random();
        }


        //TODO: make this more efficient. robust, pooling, memory managment, etc.
        public string GenerateSessionCode()
        {
            return _random.Next(100000, 999999).ToString();
        }

        public async Task<bool> CodeExistsAsync(string code)
        {
            return await _dbContext.RobotSessions.AnyAsync(rs => rs.SessionCode == code);
        }

        public async Task<Guid?> GetSessionIdByCodeAsync(string code)
        {
            var session = await _dbContext.RobotSessions.FirstOrDefaultAsync(rs => rs.SessionCode == code);
            return session?.Id;
        }
    }
}
