// bloom
// InMemoryRobotStateRepository.cs
// Thread-safe in-memory storage for current robot states within sessions.
// Every state change is also persisted as a RobotStateHistory record in the database.
// Created: 11/18/2025

using System.Collections.Concurrent;
using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Repositories
{
    public class InMemoryRobotStateRepository : IRobotStateRepository
    {
        // Dictionary structure: sessionId -> (robotId -> RobotState)
        private readonly ConcurrentDictionary<string, ConcurrentDictionary<Guid, RobotState>> _robotStates;
        private readonly ConcurrentDictionary<string, ConcurrentDictionary<Guid, int>> _stateChangeCounters;
        private readonly IServiceScopeFactory _scopeFactory;
        private readonly ILogger<InMemoryRobotStateRepository> _logger;

        public InMemoryRobotStateRepository(
            IServiceScopeFactory scopeFactory,
            ILogger<InMemoryRobotStateRepository> logger)
        {
            _robotStates = new ConcurrentDictionary<string, ConcurrentDictionary<Guid, RobotState>>();
            _stateChangeCounters = new ConcurrentDictionary<string, ConcurrentDictionary<Guid, int>>();
            _scopeFactory = scopeFactory;
            _logger = logger;
        }

        public void Add(string sessionId, RobotState state)
        {
            if (string.IsNullOrEmpty(sessionId) || state == null)
                throw new ArgumentException("Session ID and state cannot be null or empty");

            var sessionStates = _robotStates.GetOrAdd(sessionId, _ => new ConcurrentDictionary<Guid, RobotState>());
            var sessionCounters = _stateChangeCounters.GetOrAdd(sessionId, _ => new ConcurrentDictionary<Guid, int>());

            sessionStates.AddOrUpdate(state.Id, state, (_, _) => state);
            sessionCounters.AddOrUpdate(state.Id, 1, (_, count) => count + 1);

            _ = LogToDbAsync(sessionId, state);
        }

        public RobotState? Get(string sessionId, Guid id)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_robotStates.TryGetValue(sessionId, out var sessionStates))
            {
                sessionStates.TryGetValue(id, out var state);
                return state;
            }

            return null;
        }

        public IEnumerable<RobotState> GetAll(string sessionId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_robotStates.TryGetValue(sessionId, out var sessionStates))
                return sessionStates.Values.ToList();

            return Enumerable.Empty<RobotState>();
        }

        public IEnumerable<RobotState> GetAllCurrentStates()
        {
            var allStates = new List<RobotState>();
            foreach (var sessionStates in _robotStates.Values)
                allStates.AddRange(sessionStates.Values);
            return allStates;
        }

        public void Remove(string sessionId, Guid id)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_robotStates.TryGetValue(sessionId, out var sessionStates))
            {
                sessionStates.TryRemove(id, out _);
                if (_stateChangeCounters.TryGetValue(sessionId, out var counters))
                    counters.TryRemove(id, out _);
            }
        }

        public void ClearSession(string sessionId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            _robotStates.TryRemove(sessionId, out _);
            _stateChangeCounters.TryRemove(sessionId, out _);
        }

        public int GetStateChangeCount(string sessionId, Guid robotId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_stateChangeCounters.TryGetValue(sessionId, out var counters))
            {
                counters.TryGetValue(robotId, out var count);
                return count;
            }

            return 0;
        }

        public void ResetStateChangeCount(string sessionId, Guid robotId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_stateChangeCounters.TryGetValue(sessionId, out var counters))
                counters.AddOrUpdate(robotId, 0, (_, _) => 0);
        }

        private async Task LogToDbAsync(string sessionId, RobotState state)
        {
            if (!Guid.TryParse(sessionId, out var sessionGuid))
            {
                _logger.LogWarning("Skipping DB log: sessionId '{SessionId}' is not a valid Guid", sessionId);
                return;
            }

            try
            {
                using var scope = _scopeFactory.CreateScope();
                var db = scope.ServiceProvider.GetRequiredService<BloomDbContext>();

                db.RobotStateHistorys.Add(new RobotStateHistory
                {
                    RobotSessionId = sessionGuid,
                    // Snapshot strips the navigation property to avoid EF tracking the Behavior entity
                    RobotState = Snapshot(state),
                    Timestamp = DateTime.UtcNow
                });

                await db.SaveChangesAsync();
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to persist RobotStateHistory for session {SessionId}", sessionId);
            }
        }

        private static RobotState Snapshot(RobotState state) => new()
        {
            Id = state.Id,
            RobotId = state.RobotId,
            Status = state.Status,
            CurrentTask = state.CurrentTask,
            CurrentBehaviorId = state.CurrentBehaviorId,
            LastStatusChange = state.LastStatusChange,
            SpeechLog = state.SpeechLog
        };
    }
}
