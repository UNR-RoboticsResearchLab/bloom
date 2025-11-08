
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
    }
    
}