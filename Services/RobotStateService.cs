


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

        public async Task<ICollection<RobotState>> GetAllCurrentRobotStatesAsync()
        {
            var robotStates = await _dbContext.RobotStates.ToListAsync();

            return robotStates;
        }

        public async Task<RobotState> GetCurrentRobotStateByRobotIdAsync(string robotId)
        {
            
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

        public bool UpdateState(RobotStateDto robotState)
        {
            throw new NotImplementedException();
        }
    }
}