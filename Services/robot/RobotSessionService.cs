// bloom
// RobotSessionService.cs
// Service layer for managing robot sessions with dual in-memory/database storage
// In-memory storage for current states (real-time access), database for historical snapshots (every 10 changes)
// Created: 11/18/2025

using bloom.Models;
using bloom.Data;
using bloom.Repositories;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class RobotSessionService : IRobotSessionService
    {
        private readonly BloomDbContext _dbContext;
        private readonly IRobotSessionRepository _sessionRepository;
        private readonly IRobotStateRepository _stateRepository;
        private readonly ISessionCodeService _sessionCodeService;

        public RobotSessionService(
            BloomDbContext dbContext,
            IRobotSessionRepository sessionRepository,
            IRobotStateRepository stateRepository,
            ISessionCodeService sessionCodeService)
        {
            _dbContext = dbContext;
            _sessionRepository = sessionRepository;
            _stateRepository = stateRepository;
            _sessionCodeService = sessionCodeService;
        }

        public async Task<RobotSession> StartSessionAsync(string? userId = null, bool anon = false)
        {

            var sessionId = Guid.NewGuid();

            var session = new RobotSession
            {
                UserId = anon ? null : userId,
                CreatedAt = DateTime.UtcNow,
                LastUpdatedAt = DateTime.UtcNow,
                Robots = 0,
                SessionCode = await GenerateSessionCodeAsync()
            };

            await _sessionRepository.AddAsync(session);

            return session;
        }

        public async Task EndSessionAsync(Guid sessionId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            // Archive any remaining active states before clearing
            var activeStates = _stateRepository.GetAll(sessionId.ToString());
            foreach (var state in activeStates)
            {
                await _sessionRepository.AddStateHistoryAsync(sessionId, state);
            }

            // Clear in-memory states
            _stateRepository.ClearSession(sessionId.ToString());

            // Update session metadata
            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task AddRobotToSessionAsync(Guid sessionId, RobotState robotState)
        {
            if (robotState == null)
                throw new ArgumentNullException(nameof(robotState));

            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            // Add to in-memory storage
            _stateRepository.Add(sessionId.ToString(), robotState);

            // Increment robot count
            session.Robots += 1;
            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task UpdateRobotStateAsync(Guid sessionId, Guid robotId, RobotState newState)
        {
            if (newState == null)
                throw new ArgumentNullException(nameof(newState));

            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var sessionIdStr = sessionId.ToString();

            // Update in-memory state
            _stateRepository.Add(sessionIdStr, newState);

            // Check if we should archive (every 10th change)
            int changeCount = _stateRepository.GetStateChangeCount(sessionIdStr, robotId);
            if (changeCount % 10 == 0)
            {
                // Archive current state to database
                await _sessionRepository.AddStateHistoryAsync(sessionId, newState);
                _stateRepository.ResetStateChangeCount(sessionIdStr, robotId);
            }

            // Update session timestamp
            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task RemoveRobotFromSessionAsync(Guid sessionId, Guid robotId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var sessionIdStr = sessionId.ToString();
            var lastState = _stateRepository.Get(sessionIdStr, robotId);

            // Archive the last state before removing
            if (lastState != null)
            {
                await _sessionRepository.AddStateHistoryAsync(sessionId, lastState);
            }

            // Remove from in-memory storage
            _stateRepository.Remove(sessionIdStr, robotId);

            // Decrement robot count
            if (session.Robots > 0)
                session.Robots -= 1;

            session.LastUpdatedAt = DateTime.UtcNow;
            _dbContext.Update(session);
            await _dbContext.SaveChangesAsync();
        }

        public async Task<IEnumerable<RobotState>> GetCurrentStatesAsync(Guid sessionId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            return _stateRepository.GetAll(sessionId.ToString());
        }

        public async Task<IEnumerable<Guid>> GetSessionRobotsAsync(Guid sessionId)
        {
            var session = await _sessionRepository.GetAsync(sessionId);
            if (session == null)
                throw new KeyNotFoundException($"RobotSession with ID {sessionId} not found");

            var states = _stateRepository.GetAll(sessionId.ToString());
            return states.Select(s => s.RobotId).Distinct().ToList();
        }

        public async Task<IEnumerable<RobotStateHistory>> GetSessionHistoryAsync(Guid sessionId)
        {
            return await _sessionRepository.GetHistoryAsync(sessionId);
        }

        public async Task<RobotSession?> GetSessionAsync(Guid sessionId)
        {
            return await _sessionRepository.GetAsync(sessionId);
        }

        public async Task<IEnumerable<RobotSession>> GetAllSessionsAsync()
        {
            return await _sessionRepository.GetAllAsync();
        }

        public async Task<string> GenerateSessionCodeAsync()
        {
            
            string code;
            int attempts = 0;
            const int maxAttempts = 10;

            // Generate unique code with retry logic
            do
            {
                code = _sessionCodeService.GenerateSessionCode();
                if (!await _sessionCodeService.CodeExistsAsync(code))
                    break;
                attempts++;
            } while (attempts < maxAttempts);

            if (attempts >= maxAttempts)
                throw new InvalidOperationException("Failed to generate unique session code after maximum attempts");

            return code;
        }

        public async Task<RobotSession?> GetSessionByCodeAsync(string code)
        {
            return await _dbContext.RobotSessions.FirstOrDefaultAsync(rs => rs.SessionCode == code);
        }
    }
}
