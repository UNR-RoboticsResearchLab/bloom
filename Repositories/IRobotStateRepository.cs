
using bloom.Models;

namespace bloom.Repositories
{
    public interface IRobotStateRepository
    {
        void Add(string sessionId, RobotState state);
        RobotState? Get(string sessionId, Guid id);
        IEnumerable<RobotState> GetAll(string sessionId);
        void Remove(string sessionId, Guid id);
        void ClearSession(string sessionId);
        int GetStateChangeCount(string sessionId, Guid robotId);
        void ResetStateChangeCount(string sessionId, Guid robotId);

        /// <summary>
        /// Gets all current robot states across all active sessions
        /// </summary>
        /// <returns>Collection of all active RobotStates</returns>
        IEnumerable<RobotState> GetAllCurrentStates();
    }

}