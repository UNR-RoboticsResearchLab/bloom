
using bloom.Models;

namespace bloom.Services
{
    public interface IRobotService
    {
        Task<Robot> RegisterRobotAsync(RegisterRobotDto robotDto);
        Task<string> GetRobotFirmwareVersionAsync(int robotId);
        Task<IEnumerable<Robot>> GetAllRobotsAsync();
        Task<Robot?> GetRobotByIdAsync(int robotId);
        Task<Account?> GetUserByRobotIdAsync(int robotId);
        Task<IEnumerable<Robot>> GetRobotByRegisteredUserIdAsync(string userId);
    }
}