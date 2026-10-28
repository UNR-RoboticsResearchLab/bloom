// bloom
// IRobotService.cs
// Interface defining robot service operations to manage robot states and needed functionalities
// Created: 11/1/2025

using System;
using System.Collections.Generic;
using System.Security.Claims;
using System.Threading.Tasks;
using bloom.Models;
using bloom.Models.dto;

namespace bloom.Services
{
    public interface IRobotService
    {
        /**
         * <summary> 
         * GetRobotByUserIdAsync() gets all of the robots currently associated with a given user.
         * </summary>
         * <returns>List of robots</returns>
        */
        Task<ICollection<Robot>> GetRobotsByUserIdAsync(string userId);

        /**
         * <summary>
         * GetRobotsByFirmwareVersion() gets all of the robots with a specific firmware version.
         * </summary>
         * <returns>List of Robots</returns>
         */
        Task<ICollection<Robot>> GetRobotsByFirmwareVersion(string firmwareVersion);

        /**
         * <summary>
         * GetAllRobotsAsync() gets all of the robots currently registered in the system.
         * </summary>
         * <returns>List of robots</returns>
        */
        Task<ICollection<Robot>> GetAllRobotsAsync();

        /**
         * <summary>
         * RegisterRobotAsync() registers a new robot in the system. Admin-only; pair with IssueSecretAsync to provision it.
         * </summary>
         * <returns>Guid Id</returns>
         */
        Task<Guid> RegisterRobotAsync(RobotDto robot);

        /**
         * <summary>
         * UpdateRobotAsync() updates an existing robot's information.
         * </summary>
         * <returns>bool</returns>
         */
        Task<bool> UpdateRobotAsync(Guid robotId, RobotDto robot);

        /**
         * <summary>
         * DeleteRobotAsync() deletes a robot from the system.
         * </summary>
         * <returns>bool</returns>
         */
        Task<bool> DeleteRobotAsync(Guid robotId);

        /**
         * <summary>
         * GetRobotByIdAsync() gets a specific robot by its ID.
         * </summary>
         * <returns>Robot or null</returns>
         */
        Task<Robot?> GetRobotByIdAsync(Guid robotId);

        /**
         * <summary>
         * LoginRobotAsync() authenticates a robot by RobotId + Secret and returns a signed JWT.
         * Name-only login is accepted only while Robot:AllowLegacyNameOnlyLogin is true, and only
         * for robots that have no secret yet.
         * </summary>
         * <returns>Token and robot ID, or null if the credentials don't match</returns>
         */
        Task<RobotLoginResult?> LoginRobotAsync(RobotLoginDto login);

        /**
         * <summary>
         * IssueSecretAsync() generates a new login secret for a robot, replacing any previous one
         * and invalidating every token issued before it. The plaintext is returned once and never stored.
         * </summary>
         * <returns>The new secret, or null if the robot doesn't exist</returns>
         */
        Task<string?> IssueSecretAsync(Guid robotId);

        /**
         * <summary>
         * RevokeCredentialsAsync() removes a robot's secret and invalidates all its tokens.
         * </summary>
         * <returns>false if the robot doesn't exist</returns>
         */
        Task<bool> RevokeCredentialsAsync(Guid robotId);

        /**
         * <summary>
         * IsRobotTokenCurrentAsync() checks a validated robot JWT's "tv" claim against the robot's
         * current TokenVersion, so rotate/revoke takes effect before the token expires.
         * </summary>
         */
        Task<bool> IsRobotTokenCurrentAsync(ClaimsPrincipal principal);

        /**
         * <summary>
         * GetAvailableBehaviorsAsync() returns all unique behavior values found in LessonStep.Behaviors.
         * </summary>
         * <returns>Sorted list of distinct behavior name strings</returns>
         */
        Task<ICollection<Behavior>> GetAvailableBehaviorsAsync();

        /**
         * <summary>
         * GetAvailableMotorSequencesAsync() returns all unique motor sequence values from LessonStep.MotorSequence.
         * </summary>
         * <returns>Sorted list of distinct motor sequence name strings</returns>
         */
        Task<ICollection<string>> GetAvailableMotorSequencesAsync();
    }

    public record RobotLoginResult(string Token, Guid RobotId);
}
