// bloom
// InMemoryRobotStateRepository.cs
// Thread-safe in-memory storage for current robot states within sessions
// Stores only active states; historical snapshots are persisted to database
// Created: 11/18/2025

using System.Collections.Concurrent;
using bloom.Models;

namespace bloom.Repositories
{
    public class InMemoryRobotStateRepository : IRobotStateRepository
    {
        // Dictionary structure: sessionId -> (robotId -> RobotState)
        private readonly ConcurrentDictionary<string, ConcurrentDictionary<Guid, RobotState>> _robotStates;

        // Tracks state changes per robot to trigger archival every 10th change
        // Structure: sessionId -> (robotId -> change count)
        private readonly ConcurrentDictionary<string, ConcurrentDictionary<Guid, int>> _stateChangeCounters;

        public InMemoryRobotStateRepository()
        {
            _robotStates = new ConcurrentDictionary<string, ConcurrentDictionary<Guid, RobotState>>();
            _stateChangeCounters = new ConcurrentDictionary<string, ConcurrentDictionary<Guid, int>>();
        }

        public void Add(string sessionId, RobotState state)
        {
            if (string.IsNullOrEmpty(sessionId) || state == null)
                throw new ArgumentException("Session ID and state cannot be null or empty");

            var sessionStates = _robotStates.GetOrAdd(sessionId, _ => new ConcurrentDictionary<Guid, RobotState>());
            var sessionCounters = _stateChangeCounters.GetOrAdd(sessionId, _ => new ConcurrentDictionary<Guid, int>());

            sessionStates.AddOrUpdate(state.Id, state, (_, _) => state);
            sessionCounters.AddOrUpdate(state.Id, 1, (_, count) => count + 1);
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
            {
                return sessionStates.Values.ToList();
            }

            return Enumerable.Empty<RobotState>();
        }

        public IEnumerable<RobotState> GetAllCurrentStates()
        {
            var allStates = new List<RobotState>();

            // Thread-safe: ConcurrentDictionary.Values provides snapshot enumeration
            foreach (var sessionStates in _robotStates.Values)
            {
                // Each sessionStates is also a ConcurrentDictionary (thread-safe)
                allStates.AddRange(sessionStates.Values);
            }

            return allStates;
        }

        public void Remove(string sessionId, Guid id)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_robotStates.TryGetValue(sessionId, out var sessionStates))
            {
                sessionStates.TryRemove(id, out _);
                _stateChangeCounters.TryGetValue(sessionId, out var counters);
                counters?.TryRemove(id, out _);
            }
        }

        public void ClearSession(string sessionId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            _robotStates.TryRemove(sessionId, out _);
            _stateChangeCounters.TryRemove(sessionId, out _);
        }

        /// <summary>
        /// Gets the state change count for a robot in a session. Used to determine when to archive to database.
        /// </summary>
        /// <returns>Change count, or 0 if robot not found</returns>
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

        /// <summary>
        /// Resets the state change counter for a robot (called after archiving to database)
        /// </summary>
        public void ResetStateChangeCount(string sessionId, Guid robotId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty");

            if (_stateChangeCounters.TryGetValue(sessionId, out var counters))
            {
                counters.AddOrUpdate(robotId, 0, (_, _) => 0);
            }
        }
    }
}