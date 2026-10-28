// bloom
// RobotService.cs
// Service implementation for database services that will be used for managing robot state
// Created: 11/1/25

using System.IdentityModel.Tokens.Jwt;
using System.Security.Claims;
using System.Security.Cryptography;
using System.Text;
using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.IdentityModel.Tokens;

namespace bloom.Services
{
    public class RobotService : IRobotService
    {
        private readonly IAccountService _accountService;
        private readonly BloomDbContext _dbContext;
        private readonly IConfiguration _configuration;
        private static readonly PasswordHasher<Robot> _secretHasher = new();

        // JWT claim names shared with the token-version check in Program.cs and the shell relay.
        public const string TokenVersionClaim = "tv";
        public const string CredentialClaim = "cred";

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

        public async Task<RobotLoginResult?> LoginRobotAsync(RobotLoginDto loginDto)
        {
            Robot? robot;
            bool viaSecret;

            if (!string.IsNullOrEmpty(loginDto.Secret))
            {
                if (loginDto.RobotId is not Guid robotId)
                    return null;

                robot = await _dbContext.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
                if (robot?.SecretHash == null)
                    return null;

                var verify = _secretHasher.VerifyHashedPassword(robot, robot.SecretHash, loginDto.Secret);
                if (verify == PasswordVerificationResult.Failed)
                    return null;
                viaSecret = true;
            }
            else
            {
                // Rollout mode only: robots that haven't been provisioned with a secret yet. Their
                // tokens carry no "tv" claim, so the shell relay refuses them.
                if (!AllowLegacyNameOnlyLogin || string.IsNullOrEmpty(loginDto.Name))
                    return null;

                robot = await _dbContext.Robots.FirstOrDefaultAsync(r => r.Name == loginDto.Name);
                if (robot == null || robot.SecretHash != null)
                    return null;
                viaSecret = false;
            }

            // Keep the stored IP current — robots can change address across reboots.
            bool hasRealIp = !string.IsNullOrWhiteSpace(loginDto.IPAddress)
                             && loginDto.IPAddress != "N/A";
            if (hasRealIp && robot.IPAddress != loginDto.IPAddress)
            {
                robot.IPAddress = loginDto.IPAddress;
                await _dbContext.SaveChangesAsync();
            }

            return new RobotLoginResult(GenerateRobotJwt(robot, viaSecret), robot.Id);
        }

        public async Task<string?> IssueSecretAsync(Guid robotId)
        {
            var robot = await _dbContext.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
            if (robot == null)
                return null;

            var secret = Base64UrlEncoder.Encode(RandomNumberGenerator.GetBytes(32));
            robot.SecretHash = _secretHasher.HashPassword(robot, secret);
            robot.TokenVersion++;
            await _dbContext.SaveChangesAsync();
            return secret;
        }

        public async Task<bool> RevokeCredentialsAsync(Guid robotId)
        {
            var robot = await _dbContext.Robots.FirstOrDefaultAsync(r => r.Id == robotId);
            if (robot == null)
                return false;

            robot.SecretHash = null;
            robot.TokenVersion++;
            await _dbContext.SaveChangesAsync();
            return true;
        }

        public async Task<bool> IsRobotTokenCurrentAsync(ClaimsPrincipal principal)
        {
            if (!Guid.TryParse(principal.FindFirst("robotId")?.Value, out var robotId))
                return false;

            var tvClaim = principal.FindFirst(TokenVersionClaim)?.Value;
            if (tvClaim == null)
                return AllowLegacyNameOnlyLogin;

            if (!int.TryParse(tvClaim, out var tv))
                return false;

            var current = await _dbContext.Robots
                .Where(r => r.Id == robotId)
                .Select(r => (int?)r.TokenVersion)
                .FirstOrDefaultAsync();
            return current == tv;
        }

        private bool AllowLegacyNameOnlyLogin =>
            bool.TryParse(_configuration["Robot:AllowLegacyNameOnlyLogin"], out var allow) && allow;

        public async Task<ICollection<Behavior>> GetAvailableBehaviorsAsync()
        {
            return await _dbContext.Behaviors
                .OrderBy(b => b.Name)
                .ToListAsync();
        }

        public async Task<ICollection<string>> GetAvailableMotorSequencesAsync()
        {
            var sequences = await _dbContext.LessonSteps
                .Where(s => s.MotorSequence != null && s.MotorSequence != "")
                .Select(s => s.MotorSequence!)
                .Distinct()
                .OrderBy(s => s)
                .ToListAsync();

            return sequences;
        }

        private string GenerateRobotJwt(Robot robot, bool viaSecret)
        {
            var jwtKey = _configuration["Jwt:Key"]
                ?? throw new InvalidOperationException("Jwt:Key is not configured.");
            var expireMinutes = int.TryParse(_configuration["Jwt:ExpireMinutes"], out var m) ? m : 30;

            var claims = new List<Claim>
            {
                new Claim(JwtRegisteredClaimNames.Sub, robot.Id.ToString()),
                new Claim(JwtRegisteredClaimNames.Jti, Guid.NewGuid().ToString()),
                new Claim("robotId", robot.Id.ToString()),
                new Claim("robotName", robot.Name),
                new Claim(ClaimTypes.Role, "Robot"),
                new Claim(CredentialClaim, viaSecret ? "secret" : "legacy"),
            };
            if (viaSecret)
                claims.Add(new Claim(TokenVersionClaim, robot.TokenVersion.ToString()));

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