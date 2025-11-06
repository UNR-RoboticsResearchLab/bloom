// bloom
// RobotService.cs
// Service implementation for database services that will be used for managing robot state
// Created: 11/1/25

using bloom.Data;
using bloom.Models;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class RobotService : IRobotService
    {
        private readonly IAccountService _accountService;
        private readonly BloomDbContext _dbContext;
        
        public RobotService (IAccountService accountService, BloomDbContext context)
        {
            _accountService = accountService;
            _dbContext = context;
        }

        public Task<RobotState> GetRobotStateAsync(string robotId)
        {
            throw new NotImplementedException();
        }


        public Task<Assignment> GetCurrentAssignmentAsync(string robotId)
        {
            throw new NotImplementedException();
        }

        public Task UpdateRobotStateAsync(string robotId, RobotState state)
        {
            throw new NotImplementedException();
        }

        public Task AssignTaskAsync(string robotId, Assignment assignment)
        {
            throw new NotImplementedException();
        }

        public Task UpdateAssignmentStatusAsync(string robotId, string assignmentId, Assignment status)
        {
            throw new NotImplementedException();
        }

        public Task<IEnumerable<string>> GetAvailableRobotsAsync()
        {
            throw new NotImplementedException();
        }

        public Task SyncRobotStateAsync(string robotId)
        {
            throw new NotImplementedException();
        }

        public Task RegisterRobotAsync(string robotId, RobotConfigDto config)
        {
            throw new NotImplementedException();
        }
    }
}