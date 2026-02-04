// bloom
// RobotService.cs
// Service implementation for database services that will be used for managing robot state
// Created: 11/1/25

using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
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

        public async Task<ICollection<Robot>> GetAllRobotsAsync()
        {
            var robots = await _dbContext.Robots.ToListAsync();

            return robots;
        }

        public async Task<ICollection<Robot>> GetRobotsByFirmwareVersion(string firmwareVersion)
        {
            var robots = await _dbContext.Robots.Where(r => r.FirmwareVersion == firmwareVersion).ToListAsync();

            return robots;
        }

        public async Task<ICollection<Robot>> GetRobotsByUserIdAsync(string userId)
        {
            var robots = await _dbContext.Robots.Where(r => r.RegisteredUserId == userId).ToListAsync();

            return robots;
        }

        public async Task<Guid> RegisterRobotAsync(RobotDto robot)
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

            await _dbContext.Robots.AddAsync(newRobot);

            try
            {
                await _dbContext.SaveChangesAsync();
                return newRobot.Id;
            }
            catch(Exception ex)
            {
                Console.WriteLine($"Exception occured on service RobotService: {ex.Message}");
                return Guid.Empty;
            }
        }

        public async Task<bool> UpdateRobotAsync(Guid robotId, RobotDto robot)
        {
            var existingRobot = await _dbContext.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
            if (existingRobot == null)
            {
                return false;
            }

            existingRobot.Name = robot.Name;
            existingRobot.Model = robot.Model;
            existingRobot.SerialNumber = robot.SerialNumber;
            existingRobot.ManufactureDate = robot.ManufactureDate;
            existingRobot.FirmwareVersion = robot.FirmwareVersion;
            existingRobot.IPAddress = robot.IPAddress;
            existingRobot.RegisteredUserId = robot.RegisteredUserId;

            try
            {
                await _dbContext.SaveChangesAsync();
                return true;
            }
            catch(Exception ex)
            {
                Console.WriteLine($"Exception occurred on service RobotService: {ex.Message}");
                return false;
            }
        }

        public async Task<bool> DeleteRobotAsync(Guid robotId)
        {
            var robot = await _dbContext.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
            if (robot == null)
            {
                return false;
            }

            _dbContext.Robots.Remove(robot);

            try
            {
                await _dbContext.SaveChangesAsync();
                return true;
            }
            catch(Exception ex)
            {
                Console.WriteLine($"Exception occurred on service RobotService: {ex.Message}");
                return false;
            }
        }

        public async Task<Robot?> GetRobotByIdAsync(Guid robotId)
        {
            return await _dbContext.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
        }
    }
}