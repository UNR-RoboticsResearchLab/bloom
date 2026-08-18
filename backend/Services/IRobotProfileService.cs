// bloom
// IRobotProfileService.cs
// Interface defining operations for managing a student's per-account robot
// customization (nickname, personality, etc.).
// Created: 08/17/2026

using System.Collections.Generic;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public interface IRobotProfileService
    {
        /**
         * <summary>
         * GetByAccountIdAsync() returns the robot profile for the given account, or null if none exists yet.
         * </summary>
         */
        Task<RobotProfile?> GetByAccountIdAsync(string accountId);

        /**
         * <summary>
         * UpsertAsync() creates or updates the robot profile for the given account.
         * </summary>
         * <exception cref="ArgumentException">Thrown when PersonalityTrait is not a recognized preset.</exception>
         */
        Task<RobotProfile> UpsertAsync(string accountId, RobotProfileDto dto);

        /**
         * <summary>
         * GetPersonalityPresets() returns the fixed list of selectable personality traits.
         * </summary>
         */
        ICollection<string> GetPersonalityPresets();
    }
}
