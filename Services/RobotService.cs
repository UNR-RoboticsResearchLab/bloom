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

        public ICollection<Robot> GetAllRobotsAsync()
        {
            var robots = _dbContext.Robots.ToList();

            return robots;
        }

        public ICollection<Robot> GetRobotsByFirmwareVersion(string firmwareVersion)
        {
            var robots = _dbContext.Robots.Where(r => r.FirmwareVersion == firmwareVersion).ToList();

            return robots;
        }

        public ICollection<Robot> GetRobotsByUserIdAsync(string userId)
        {
            var robots = _dbContext.Robots.Where(r => r.RegisteredUserId == userId).ToList();

            return robots;
        }

        public bool RegisterRobotAsync(RobotDto robot)
        {
            if (robot.Name == null)
            {
                throw new Exception("Robot name is invalid");
            }
            if (robot.IPAddress == null)
            {
                throw new Exception("Robot IP is invalid");
            }

            var newRobot = new Robot
            {
                Name = robot.Name,
                Model = robot.Model,
                SerialNumber = robot.SerialNumber,
                ManufactureDate = robot.ManufactureDate,
                FirmwareVersion = robot.FirmwareVersion,
                IPAddress = robot.IPAddress,
                RegisteredUserId = robot.RegisteredUserId
            };

            _dbContext.Robots.Add(newRobot);

            try
            {
                _dbContext.SaveChangesAsync();
                return true;
            }
            catch(Exception ex)
            {
                Console.WriteLine($"Exception occured on service RobotService: {ex.Message}");
                return false;
            }
        }
    }
}