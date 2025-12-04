
using bloom.Models;

namespace bloom.Repositories
{
    public interface IRobotRepository
    {
        Task<Robot?> GetAsync(Guid id);
        Task AddAsync(Robot robot);
        Task<IEnumerable<RobotStateHistory>> GetHistoryAsync(Guid sessionId);

    }
}