// bloom
// IRobotService.cs
// Interface defining robot service operations to manage robot states and needed functionalities
// Created: 11/1/2025

using System;
using System.Collections.Generic;
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
         * RegisterRobotAsync() registers a new robot in the system. Should be triggered on first boot.
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
    }
}