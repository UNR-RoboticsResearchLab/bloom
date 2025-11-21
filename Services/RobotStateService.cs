


using System.Threading.Tasks;
using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    class RobotStateService : IRobotStateService
    {
        private readonly BloomDbContext _dbContext;
        
        public RobotStateService(BloomDbContext context)
        {
            _dbContext = context;
        }

        public ICollection<RobotState> GetAllCurrentRobotStatesAsync()
        {
            // Note: RobotState is an owned entity type and cannot be queried directly from DbContext
            // TODO: Implement properly by querying through a parent entity that owns RobotState
            throw new NotImplementedException("RobotState is an owned entity and must be queried through its owner");
        }

        public RobotState GetCurrentRobotStateByRobotIdAsync(string robotId)
        {
            throw new NotImplementedException();
        }

        public RobotState GetCurrentRobotStateBySessionIdAsync(string sessionId)
        {
            throw new NotImplementedException();
        }

        public ICollection<RobotState> GetCurrentRobotStatesByClassroomIdAsync(string classroomId)
        {
            throw new NotImplementedException();
        }

        public ICollection<RobotState> GetRobotStatesBySessionId(string sessionId)
        {
            throw new NotImplementedException();
        }
    }
}