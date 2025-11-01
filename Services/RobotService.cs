// bloom
// RobotService.cs
// Implementation of a robot service

using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using bloom.Models;
using Microsoft.EntityFrameworkCore;
using bloom.Data;

namespace bloom.Services
{
    public class RobotService : IRobotService, IDisposable
    {
        private readonly ILogger<RobotService> _logger;
        private readonly BloomDbContext _context;
        private CancellationTokenSource _cts = new CancellationTokenSource();
        private bool _disposed;

        public RobotService(ILogger<RobotService> logger, BloomDbContext context)
        {
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
            _context = context ?? throw new ArgumentNullException(nameof(context));
        }

        public async Task<Robot> RegisterRobotAsync(RegisterRobotDto robotDto)
        {
            _logger.LogInformation($"Registering robot with serial number: {robotDto.SerialNumber}");
            
            var robot = new Robot
            {
                Name = robotDto.Name,
                Model = robotDto.Model,
                SerialNumber = robotDto.SerialNumber,
                ManufactureDate = robotDto.ManufactureDate,
                FirmwareVersion = robotDto.FirmwareVersion,
                IPAddress = robotDto.IPAddress,
                RegisteredUserId = robotDto.RegisteredUserId
            };

            _context.Robots.Add(robot);
            await _context.SaveChangesAsync();
            return robot;
        }

        public async Task<string> GetRobotFirmwareVersionAsync(int robotId)
        {
            var robot = await _context.Robots.FindAsync(robotId);
            
            if (robot == null)
            {
                throw new KeyNotFoundException($"Robot with ID {robotId} not found.");
            }

            return robot.FirmwareVersion;
        }

        public async Task<IEnumerable<Robot>> GetAllRobotsAsync()
        {
            return await _context.Robots
                .Include(r => r.RegisteredUser)
                .ToListAsync();
        }

        public async Task<Robot?> GetRobotByIdAsync(int robotId)
        {
            return await _context.Set<Robot>()
                .Include(r => r.RegisteredUser)
                .FirstOrDefaultAsync(r => r.Id == robotId);
        }

        public async Task<Account?> GetUserByRobotIdAsync(int robotId)
        {
            var robot = await _context.Set<Robot>()
                .Include(r => r.RegisteredUser)
                .FirstOrDefaultAsync(r => r.Id == robotId);

            return robot?.RegisteredUser;
        }

        public async Task<IEnumerable<Robot>?> GetRobotByRegisteredUserIdAsync(string userId)
        {
            return await _context.Set<Robot>()
                .Include(r => r.RegisteredUser)
                .Where(r => r.RegisteredUserId == userId)
                .ToListAsync();
        }

        public void Dispose()
        {
            if (_disposed) return;
            _cts?.Dispose();
            _disposed = true;
            GC.SuppressFinalize(this);
        }
    }
}