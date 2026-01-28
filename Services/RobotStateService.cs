using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using bloom.Repositories;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    class RobotStateService : IRobotStateService
    {
        private readonly IRobotStateRepository _stateRepository;

        public RobotStateService(IRobotStateRepository stateRepository)
        {
            _stateRepository = stateRepository;
        }

        public ICollection<RobotState> GetAllCurrentRobotStatesAsync()
        {
            // Note: Despite the "Async" naming in the interface, this is a synchronous
            // in-memory operation with no I/O, so no async/await is needed
            var states = _stateRepository.GetAllCurrentStates();
            return states.ToList();
        }

        public RobotState GetCurrentRobotStateByRobotIdAsync(string robotId)
        {
            if (string.IsNullOrEmpty(robotId))
                throw new ArgumentException("Robot ID cannot be null or empty", nameof(robotId));

            if (!Guid.TryParse(robotId, out var robotGuid))
                throw new ArgumentException($"Invalid robot ID format: {robotId}", nameof(robotId));

            var allStates = _stateRepository.GetAllCurrentStates().FirstOrDefault(state => state.RobotId == robotGuid) ?? new RobotState() { Status = "Unknown", CurrentTask = "Unknown", CurrentBehaviorId = null, SpeechLog = "" };
            return allStates;
        }

        public ICollection<RobotState> GetCurrentRobotStateBySessionIdAsync(string sessionId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty", nameof(sessionId));

            var states = _stateRepository.GetAll(sessionId);
            return states.ToList();
        }

        public ICollection<RobotState> GetCurrentRobotStatesByClassroomIdAsync(string classroomId)
        {
            throw new NotImplementedException();
        }

        public ICollection<RobotState> GetRobotStatesBySessionId(string sessionId)
        {
            if (string.IsNullOrEmpty(sessionId))
                throw new ArgumentException("Session ID cannot be null or empty", nameof(sessionId));

            var states = _stateRepository.GetAll(sessionId);
            return states.ToList();
        }
    }
}