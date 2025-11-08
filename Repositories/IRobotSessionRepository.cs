

using bloom.Models;

namespace bloom.Repositories
{
    public interface IRobotSessionRepository
    {
        Task<RobotSession?> GetAsync(Guid id);
        Task AddAsync(RobotSession session);
        Task AddStateHistoryAsync(Guid sessionId, RobotState state);
        Task<IEnumerable<RobotStateHistory>> GetHistoryAsync(Guid sessionId);

    }
}