// bloom
// RobotProfileService.cs
// Service implementation for managing a student's per-account robot customization.
// Created: 08/17/2026

using bloom.Data;
using bloom.Models;
using bloom.Models.dto;
using Microsoft.EntityFrameworkCore;

namespace bloom.Services
{
    public class RobotProfileService : IRobotProfileService
    {
        private readonly BloomDbContext _dbContext;

        public RobotProfileService(BloomDbContext context)
        {
            _dbContext = context;
        }

        public async Task<RobotProfile?> GetByAccountIdAsync(string accountId)
        {
            return await _dbContext.RobotProfiles.FirstOrDefaultAsync(p => p.AccountId == accountId);
        }

        public async Task<RobotProfile> UpsertAsync(string accountId, RobotProfileDto dto)
        {
            if (!RobotPersonality.All.Contains(dto.PersonalityTrait))
            {
                throw new ArgumentException($"'{dto.PersonalityTrait}' is not a recognized personality trait.");
            }

            var profile = await _dbContext.RobotProfiles.FirstOrDefaultAsync(p => p.AccountId == accountId);

            if (profile == null)
            {
                profile = new RobotProfile
                {
                    AccountId = accountId,
                    Nickname = dto.Nickname,
                    PersonalityTrait = dto.PersonalityTrait,
                    Catchphrase = dto.Catchphrase,
                    ColorTheme = dto.ColorTheme,
                    CreatedDate = DateTime.UtcNow
                };
                await _dbContext.RobotProfiles.AddAsync(profile);
            }
            else
            {
                profile.Nickname = dto.Nickname;
                profile.PersonalityTrait = dto.PersonalityTrait;
                profile.Catchphrase = dto.Catchphrase;
                profile.ColorTheme = dto.ColorTheme;
                profile.UpdatedDate = DateTime.UtcNow;
            }

            await _dbContext.SaveChangesAsync();
            return profile;
        }

        public ICollection<string> GetPersonalityPresets()
        {
            return RobotPersonality.All;
        }
    }
}
