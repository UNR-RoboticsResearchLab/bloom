// bloom
// RobotService.cs
// Service implementation for database services that will be used for managing robot state
// Created: 11/1/25

using System.IdentityModel.Tokens.Jwt;
using System.Security.Claims;
using System.Text;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;
using Microsoft.IdentityModel.Tokens;

namespace bloom.Services
{
    public class RobotService : IRobotService
    {
        private readonly IAccountService _accountService;
        private readonly BloomDbContext _dbContext;
        private readonly IConfiguration _configuration;

        public RobotService(IAccountService accountService, BloomDbContext context, IConfiguration configuration)
        {
            _accountService = accountService;
            _dbContext = context;
            _configuration = configuration;
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

        public async Task<string?> LoginRobotAsync(RobotDto robotDto)
        {
            var robot = await _dbContext.Robots.FirstOrDefaultAsync(r =>
                r.Name == robotDto.Name && r.IPAddress == robotDto.IPAddress);

            if (robot == null)
                return null;

            return GenerateRobotJwt(robot);
        }

        private string GenerateRobotJwt(Robot robot)
        {
            var jwtKey = _configuration["Jwt:Key"]
                ?? throw new InvalidOperationException("Jwt:Key is not configured.");
            var expireMinutes = int.TryParse(_configuration["Jwt:ExpireMinutes"], out var m) ? m : 30;

            var claims = new[]
            {
                new Claim(JwtRegisteredClaimNames.Sub, robot.Id.ToString()),
                new Claim(JwtRegisteredClaimNames.Jti, Guid.NewGuid().ToString()),
                new Claim("robotId", robot.Id.ToString()),
                new Claim("robotName", robot.Name),
                new Claim(ClaimTypes.Role, "Robot"),
            };

            var key = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(jwtKey));
            var token = new JwtSecurityToken(
                issuer: _configuration["Jwt:Issuer"],
                audience: _configuration["Jwt:Audience"],
                claims: claims,
                expires: DateTime.UtcNow.AddMinutes(expireMinutes),
                signingCredentials: new SigningCredentials(key, SecurityAlgorithms.HmacSha256));

            return new JwtSecurityTokenHandler().WriteToken(token);
        }
    }
}